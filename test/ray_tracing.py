import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json

## FUNCTIONS ____________________________________
def create_cameras():
    cams = []

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    object_points = np.asarray(object.vertices).T 
    ax.scatter(object_points[0], object_points[1], object_points[2], c='green')
    #plt.show()

    with open("camera_data.json") as f:
        cam_data = json.load(f)

    for i in range(4):
        cam_info = cam_data[i]
        
        # Get Matricies
        ex_mat = np.array(cam_info["extrinsic_mat"], dtype=np.float32)
        in_mat = np.array(cam_info["intrinsics"]["intrinsic_matrix"], dtype=np.float32)

        cam_tmp = {
            'ex_tensor' : o3d.core.Tensor(ex_mat),
            'in_tensor' : o3d.core.Tensor(in_mat),
            'w' : cam_info["intrinsics"]["width"],
            'h' : cam_info["intrinsics"]["height"]
        }
        cams.append(cam_tmp)

        print(f"Camera_{i+1} loaded and appended")

        cam_world = np.linalg.inv(ex_mat)
        cam_pos_world = cam_world[:3,3]
        ax.scatter(cam_pos_world[0], cam_pos_world[1], cam_pos_world[2], c='red')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

    return cams

def ray_cast(scene, cams):
    ans = []
    
    for i, cam in enumerate(cams):
        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            intrinsic_matrix=cam['in_tensor'],
            extrinsic_matrix=cam['ex_tensor'],
            width_px=cam["w"],
            height_px=cam["h"]
        )

        print(f" Casting from camera{i} ...")
        ans.append(scene.cast_rays(rays)) # cast rays on scene one camera at a time
    
    return ans

def build_dict(ans_cast, object_mesh): # Added object_mesh parameter
    # should be based off the camera data but since its known its fine for now
    width_px = 1920
    height_px = 1080

    # Get triangle and vertex data from the passed object_mesh (which is already transformed)
    triangle_ids = np.asarray(object_mesh.triangles)
    vertices = np.asarray(object_mesh.vertices)
    triangle_vertices = vertices[triangle_ids]

    for i, ans in enumerate(ans_cast):
        # Initialise new array for camera
        camera_hit_list = [] # List to store hit data for the current camera

        primitive_ids = ans["primitive_ids"].numpy().reshape((height_px, width_px ))
        primitive_uvs = ans["primitive_uvs"].numpy().reshape((height_px, width_px , 2))

        count = 0

        for y in range(height_px):
            for x in range(width_px):
                tri_id = primitive_ids[y,x]

                if tri_id == o3d.t.geometry.RaycastingScene.INVALID_ID:
                    count += 1
                    continue  # Skip rays that didn't hit anything

                u, v = primitive_uvs[y, x]
                v0, v1, v2 = triangle_vertices[tri_id]
                hit_point = (1 - u - v) * v0 + u * v1 + v * v2

                dv1 = float(np.linalg.norm(hit_point - v0))
                dv2 = float(np.linalg.norm(hit_point - v1))
                dv3 = float(np.linalg.norm(hit_point - v2))

                # add data to list
                camera_hit_list.append({
                    "pixel": [x, y],
                    "triangle_id": int(tri_id),
                    "hit": hit_point.tolist(),
                    "dv1": dv1,
                    "dv2": dv2,
                    "dv3": dv3
                })

    return camera_hit_list # Return the dictionary containing all camera hit data


if __name__=="__main__":
    ## SETUP ________________________________________________

    # Load Meshes
    object = o3d.io.read_triangle_mesh("frame_0001.ply")

    # need to flip the y and z axis when converting from blender
    R_blender_to_o3d_mesh = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ], dtype=np.float64)
    object.rotate(R_blender_to_o3d_mesh, center=object.get_center())
    
    # Set Up Cameras
    cams = create_cameras()
    print("camera set up complete")

    ## RAYCAST ______________________________________________
    # # Set Up Ray Casting Scene and cast
    # scene = o3d.t.geometry.RaycastingScene()
    # scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(object))
    # cast_ans = ray_cast(scene, cams)
    # print("ray casting complete")

    # # Display Result
    # for a in cast_ans:
    #     plt.imshow(a['t_hit'].numpy())
    #     plt.show()

    # # Save Result
    # data = build_dict(cast_ans, object)
    # with open(f"gt_{1}.json", "w") as f: # change to add loop when doing all of the objects
    #     json.dump(data, f, indent=2)
    # print(f"File gt_{1} created!")
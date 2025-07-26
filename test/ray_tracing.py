import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json

## FUNCTIONS ____________________________________
def create_cameras():
    cams = []
    with open("camera_data.json") as f:
        cam_data = json.load(f)

    for i in range(4):
        cam_info = cam_data[f"Camera_{i+1}"]
        
        # Get Matricies
        ex_mat = np.array(cam_info["extrinsic_mat"], dtype=np.float32)
        in_mat = in_mat = np.array(cam_info["intrinsics"]["intrinsic_matrix"], dtype=np.float32)

        cam_tmp = {
            'ex_tensor' : o3d.core.Tensor(ex_mat),
            'in_tensor' : o3d.core.Tensor(in_mat),
            'w' : cam_info["intrinsics"]["width"],
            'h' : cam_info["intrinsics"]["height"]
        }
        cams.append(cam_tmp)

        print(f"Camera_{i+1} loaded and appended")

    return cams

def ray_cast(object, cams):

    scene = o3d.t.geometry.RaycastingScene()

    # I think this is where I adjust the location and stuff in the scene for the mesh
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(object))

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

def build_dict(ans_cast): # I really hope this function is correct - I need to properly check it later

    # should be based off the camera data but since its known its fine for now
    width_px = 1920
    height_px = 1080 

    hit_data = {}
    for i, ans in enumerate(ans_cast):
        
        # initialise new array for camera
        hit_data = {f"camera_{i}" : []}

        primitive_ids = ans["primitive_ids"].numpy().reshape((height_px, width_px ))
        primitive_uvs = ans["primitive_uvs"].numpy().reshape((height_px, width_px , 2))
        
        triangle_ids = np.asarray(object.triangles)
        vertices = np.asarray(object.vertices)
        triangle_vertices = vertices[triangle_ids]

        for y in range(height_px):
            for x in range(width_px):
                tri_id = primitive_ids[y,x]
                
                if tri_id == o3d.t.geometry.RaycastingScene.INVALID_ID:
                    continue  # Skip rays that didn't hit anything

                u, v = primitive_uvs[y, x]
                v0, v1, v2 = triangle_vertices[tri_id]
                hit_point = (1 - u - v) * v0 + u * v1 + v * v2

                dv1 = float(np.linalg.norm(hit_point - v0))
                dv2 = float(np.linalg.norm(hit_point - v1))
                dv3 = float(np.linalg.norm(hit_point - v2))
                
                # add data to dict
                hit_data[f"camera_{i}"].append({
                    "pixel": [x, y],
                    "triangle_id": int(tri_id), #oof I feel a bit iffy about this one
                    "hit": hit_point.tolist(),
                    "dv1": dv1,
                    "dv2": dv2,
                    "dv3": dv3
                })

    return hit_data


if __name__=="__main__":
    ## SETUP ________________________________________________

    # Set Up Cameras
    cams = create_cameras()
    print("camera set up complete")

    # Load Meshes
    object = o3d.io.read_triangle_mesh("frame_0001.ply")

    # Open File
    with open(f"gt_{1}.json", "w") as f: # change to add loop when doing all of the objects

    ## RAYCAST ______________________________________________
        # Set Up Ray Casting Scene and cast
        cast_ans = ray_cast(object, cams)
        print("ray casting complete")

        # Display Result
        for a in cast_ans:
            plt.imshow(a['t_hit'].numpy())
            plt.show()

        # Save Result
        data = build_dict(cast_ans)
        json.dump(data, f, indent=2)

    print(f"File gt_{1} created!")

    # might want to test loading the data and creating the point cloud with the open3d visulaisation thingy later just incase
    
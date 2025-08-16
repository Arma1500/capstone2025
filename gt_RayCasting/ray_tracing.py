import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json
import os

## FUNCTIONS ____________________________________
def look_at(cam_pos, target=np.array([0,0,0]), up=np.array([0, 0, 1])):
    # change camera extrinsic mat to look at mesh 
    forward = target - cam_pos
    forward /= np.linalg.norm(forward)
    right = np.cross(up, forward)
    right /= np.linalg.norm(right)
    true_up = np.cross(forward, right)
    
    R = np.stack([right, true_up, forward], axis=1)
    t = -R.T @ cam_pos
    
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = R.T
    extrinsic[:3, 3] = t
    
    return extrinsic

def create_cameras(target_center):
    cams = []

    # plotting camera and mesh location for debugging
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    object_points = np.asarray(object.vertices).T 
    ax.scatter(object_points[0], object_points[1], object_points[2], c='green')

    with open("gt_RayCasting/camera_data2.json") as f:
        cam_data = json.load(f)

    for i in range(4):
        cam_info = cam_data[i]
        
        # Get Matricies
        ex_mat = np.array(cam_info["extrinsic_mat"], dtype=np.float32)
        cam_world = np.linalg.inv(ex_mat)
        cam_pos_world = cam_world[:3,3]
        ex_mat_look = look_at(cam_pos_world) # need to change mat to look at object
        
        in_mat = np.array(cam_info["intrinsics"]["intrinsic_matrix"], dtype=np.float32)

        cam_tmp = {
            'ex_tensor' : o3d.core.Tensor(ex_mat_look),
            'in_tensor' : o3d.core.Tensor(in_mat),
            'w' : cam_info["intrinsics"]["width"],
            'h' : cam_info["intrinsics"]["height"]
        }
        cams.append(cam_tmp)

        print(f"Camera_{i+1} loaded and appended")
        # part of the plotting
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

        #print(f" Casting from camera {i} ...")
        
        cast_ans = scene.cast_rays(rays)
        ans.append(cast_ans)
    
    return ans

def build_dict(ans, object_mesh):
    # should be based off the camera data but since its known its fine for now
    width_px = 1920
    height_px = 1080

    # Get triangle and vertex data from the passed object_mesh (which is already transformed)
    triangle_ids = np.asarray(object_mesh.triangles)
    vertices = np.asarray(object_mesh.vertices)
    triangle_vertices = vertices[triangle_ids]
    
    # Initialise new array for camera
    camera_hit_list = []

    primitive_ids = ans['primitive_ids'].numpy().reshape((height_px, width_px))
    primitive_uvs = ans['primitive_uvs'].numpy().reshape((height_px, width_px , 2))

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

    return camera_hit_list

def load_mesh(path):
    mesh = o3d.io.read_triangle_mesh(path)
    o_center = mesh.get_center()

    # need to flip the y and z axis when converting from blender
    R_blender_to_o3d_mesh = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ], dtype=np.float64)
    mesh = mesh.rotate(R_blender_to_o3d_mesh)

    return mesh, o_center

def plot_cloud_from_json():

    point_clouds = []

    for i in range(4):
        with open(f'ground_truth/Camera_{i+1}/frame_0001.json', 'r') as f:
            camera_hit_list = json.load(f)

        # Extract just the hit points
        points = [entry['hit'] for entry in camera_hit_list if entry['hit'] is not None]

        # Convert to Point Cloud
        points_np = np.array(points, dtype=np.float32)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)

        point_clouds.append(pcd)
        print("pt cld added")

    # transform all clouds to where the first cloud is
    center = point_clouds[0].get_center()

    for i, pcd in enumerate(point_clouds):
        pcd.translate(center)

    merged_pcd = o3d.geometry.PointCloud()
    for pcd in point_clouds:
        merged_pcd += pcd

    # Visualize
    downsampled_pcd = merged_pcd.voxel_down_sample(voxel_size=0.02)
    o3d.visualization.draw_geometries([downsampled_pcd])
    o3d.io.write_point_cloud("output_pointcloud.ply", downsampled_pcd)


if __name__=="__main__":
    ## SETUP ________________________________________________
    save_count = 0

    # Load Meshes -----------------------
    mesh_folder = os.path.abspath("/home/humense/arhma-capstone/simulation/ground_truth2/mesh_dancer")
    mesh_files = sorted([f for f in os.listdir(mesh_folder) if f.endswith('.ply')])
    for mesh_file in mesh_files:
        
        if os.path.splitext(mesh_file)[0] >= "frame_0002":
            break # for debugging
                        
        mesh_path = os.path.join(mesh_folder, mesh_file)
        
        object, o_center = load_mesh(mesh_path)
        
        # Set Up Cameras ------------------
        cams = create_cameras(o_center)

        # RAYCAST ______________________________________________
        # Set Up Ray Casting Scene and cast
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(object))
        cast_ans = ray_cast(scene, cams)
        print("ray casting complete!")
        
        for i, a in enumerate(cast_ans):
            # Display Result - for debugging ------------------
            plt.imshow(a['t_hit'].numpy())
            plt.title(f"cam{i+1}")
            plt.xlabel("X-axis")
            plt.ylabel("Y-axis")
            plt.show()

            # Save Result -------------------------
            data = build_dict(a, object)
            cam_path = os.path.join(f"ground_truth/Camera_{i+1}")
            os.makedirs(cam_path, exist_ok=True)
            file_path = os.path.join(cam_path, (os.path.splitext(mesh_file)[0] + ".json"))
            with open(file_path, "w") as f: 
                json.dump(data, f, indent=2)
        
        print(f"File {file_path} saved!")
        save_count += 1 # to check how many meshes we have done
    
    print(f"{save_count} files saved")

## ----------------------------------------------------------------------------------------------------------------
    ## DISPLAY RESULT OF FIRST MESH - make sure to comment out the above code
    #plot_cloud_from_json()
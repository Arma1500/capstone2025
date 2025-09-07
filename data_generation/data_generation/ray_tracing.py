import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
import json
import os

import polyscope as ps
ps.init()

# UTILITY FUNCTIONS
# Initialise Cameras ---------------------------------------------------------------------
def read_cam_data(path):
    # read camera data
    cams = []
    with open(path) as f:
        cam_data = json.load(f)

    # convert to O3D Coordinate frame
    # -90 about x
    rot_mat = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, -1, 0, 0],
    [0, 0, 0, 1]], dtype=np.float32)

    # 180 around y - because there was trouble with the cameras facing the wrong way and I'm still not entirely sure why
    rot_y_mat = np.array([
        [-1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    # save camera data for use with o3d raycasting
    for i in range(4):
        cam_info = cam_data[i]

        #Get Matricies
        # Extrinsics
        ex_mat = np.linalg.inv(np.array(cam_info["extrinsic_mat"], dtype=np.float32))

        ex_cam_rot = rot_mat @ ex_mat # converting to o3d
        ex_cam_rot_y = ex_cam_rot @ rot_y_mat
        ex_world_rotated = np.linalg.inv(ex_cam_rot_y) #camera to world matrix - because thats what create_rays_pinhole_needs

        # Intrinsics
        in_mat = np.array(cam_info["intrinsics"]["intrinsic_matrix"], dtype=np.float32)

        cam_tmp = {
            'ex_tensor' : o3d.core.Tensor(ex_world_rotated),
            'in_tensor' : o3d.core.Tensor(in_mat),
            'w' : cam_info["intrinsics"]["width"],
            'h' : cam_info["intrinsics"]["height"]
        }
        cams.append(cam_tmp)
    return cams

# Ray Casting -----------------------------------------------------------------------------
def ray_cast(mesh, cams):

    # set up ray_casting scene
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    
    # create rays, cast the scene and save the data
    ans = []
    for cam in cams:
        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            intrinsic_matrix=cam['in_tensor'],
            extrinsic_matrix=cam['ex_tensor'],
            width_px=cam["w"],
            height_px=cam["h"]
        )
        
        cast_ans = scene.cast_rays(rays)
        ans.append(cast_ans)
    
    return ans

# Build Dictionary to save Ray Casting data ---------------------------------------------------
def build_dict(ans, mesh):
    # should be based off the camera data but since its known its fine for now
    width_px = 1920
    height_px = 1080

    # Get triangle and vertex data from the passed object_mesh (which is already transformed)
    faces = np.asarray(mesh.triangles)
    vertices = np.asarray(mesh.vertices)
    triangle_vertices = vertices[faces]

    primitive_ids = ans['primitive_ids'].numpy().reshape((height_px, width_px))
    primitive_uvs = ans['primitive_uvs'].numpy().reshape((height_px, width_px , 2))

    camera_hit_list = []
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

# VIS FUNCTIONS FOR DEBUGGING
# Polyscope scene visualisation ------------------------------------------------------------
def polyscope_cam_params(camera_data):
    ex_world = np.linalg.inv(camera_data['ex_tensor'].numpy())
    in_mat = camera_data['in_tensor'].numpy()
    w = camera_data['w']
    h = camera_data['h']

    # Intrinsics
    fy = in_mat[1, 1]
    fov_vertical_rad = 2 * math.atan(h / (2 * fy))
    fov_vertical_deg = math.degrees(fov_vertical_rad)
    aspect_ratio = w / h
    intrinsics = ps.CameraIntrinsics(fov_vertical_deg=fov_vertical_deg, aspect=aspect_ratio)

    # Extrinsics
    cam_pos = ex_world[:3, 3].tolist() # numpy array dosn't work for cam extrinsics in polyscope
    look_dir = (ex_world[:3, 2] / np.linalg.norm(ex_world[:3, 2])).tolist()
    up_dir = (ex_world[:3, 1] / np.linalg.norm(ex_world[:3, 1])).tolist()
    extrinsics = ps.CameraExtrinsics(root=cam_pos, look_dir=look_dir, up_dir=up_dir)

    return ps.CameraParameters(intrinsics, extrinsics)

# Plot from .JSON -----------------------------------------------------------------------
def plot_from_json(path):
    all_points = []

    for i in range(4):
        cam_path = os.path.join(output_path, f"Camera_{i+1}/frame_0001.json")
        with open(cam_path, 'r') as f:
            camera_hit_list = json.load(f)

        # Extract just the hit points
        hit_points = np.array([entry['hit'] for entry in camera_hit_list if entry['hit'] is not None], dtype=np.float32)

        all_points.append(hit_points)
        #print("pt cld added")

    return all_points

# MAIN FUNCTION
if __name__=="__main__":

    # File Path Set Up -------------------------------------------------------------------------
    camera_data_path = 'data_generation/data_generation/blender_camera_data.json'
    meshes_path = '/media/humense/Expansion/Capstone/meshes'

    output_path = '/media/humense/Expansion/Capstone/ground_truth'

    # Ray Casting ------------------------------------------------------------------------------
    # create cameras
    cams_data = read_cam_data(camera_data_path)

    # loop through all frames
    mesh_files = sorted([f for f in os.listdir(meshes_path) if f.endswith('.ply')])
    for mesh_file in mesh_files:
        
        if os.path.splitext(mesh_file)[0] != "frame_0002":
            break # for debugging
                        
        mesh_path = os.path.join(meshes_path, mesh_file)
        mesh = o3d.io.read_triangle_mesh(mesh_path)

        # create ray casting scene and cast
        cast_ans = ray_cast(mesh, cams_data)
        print("ray casting complete!")

        # save ray casting data in .json
        for i, a in enumerate(cast_ans):
            data = build_dict(a, mesh)

            gt_cam_paths = os.path.join(output_path, f"Camera_{i+1}")
            os.makedirs(gt_cam_paths, exist_ok=True)

            # save file
            file_path = os.path.join(gt_cam_paths, (os.path.splitext(mesh_file)[0] + ".json"))
            with open(file_path, "w") as f: 
                json.dump(data, f, indent=2)

            print(f"Ground Truth for Camera_{i+1} added to {file_path}")


    # # display for debugging ----------------------------------------------
    # # display cameras in polyscope scene
    # for i, cam in enumerate(cams_data):
    #     print(f"Camera_{i+1} --------------------------")
    #     params = polyscope_cam_params(cam)
    #     cam = ps.register_camera_view(f"Camera {i+1}", params)

    # # display original mesh in polyscope scene
    # faces = np.asarray(mesh.triangles)
    # vertices = np.asarray(mesh.vertices)
    # ps_mesh = ps.register_surface_mesh("mesh", vertices, faces)

    # # display saved ray casting result from .json file
    # hit_pts = plot_from_json(output_path)
    # for i in range(len(hit_pts)):
    #     pcd = ps.register_point_cloud(f"Ground Truth Cam{i+1}", hit_pts[i])

    # ps.show()
    # ps.remove_all_structures()

    # # display ray casting result plots
    # for i, a in enumerate(cast_ans):
    #         plt.imshow(a['t_hit'].numpy())
    #         plt.title(f"cam{i+1}")
    #         plt.xlabel("X-axis")
    #         plt.ylabel("Y-axis")
    #         plt.show()
    # # -----------------------------------------------------------------------------------




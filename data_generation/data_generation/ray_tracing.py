import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
import json
import os

import polyscope as ps
ps.init()

# Utility Functions
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

# Functions for visualisation
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

    print(f"cam_pos: {cam_pos}")
    print(f"look_dir: {look_dir}")
    print(f"up_dir: {up_dir}")

    return ps.CameraParameters(intrinsics, extrinsics)

# Plot from .JSON -----------------------------------------------------------------------
def plot_from_json(cams):
    all_points = []
    all_directions = []

    for i in range(4):
        with open(f'ground_truth_data/Camera_{i+1}/frame_0001.json', 'r') as f:
            camera_hit_list = json.load(f)

        # Extract just the hit points
        hit_points = np.array([entry['hit'] for entry in camera_hit_list if entry['hit'] is not None], dtype=np.float32)

        cam_origin = np.linalg.inv(cams[i]['ex_tensor'].numpy())[:3,3]
        direction = hit_points - cam_origin

        # Convert to Point Cloud
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(hit_points)

        all_points.append(hit_points)
        all_directions.append(direction)
        print("pt cld added")

    return all_points, all_directions 



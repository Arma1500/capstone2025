import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
import json
import cv2
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

# Normalise Depth Image ---------------------------------------------------------------
def load_normalized_depth(path, min_depth, max_depth):

    depth = cv2.imread(path, cv2.IMREAD_UNCHANGED).astype(np.float32)
    
    # the way open3d loaded the images has them upside down
    depth = cv2.flip(depth, 0)
    depth = cv2.flip(depth, 1)
    
    depth = depth[:, :, 0]

    # Normalize [0, 255] -> [0, 1]
    depth_norm = depth / 255.0
    depth = depth_norm * (max_depth - min_depth) + min_depth # Map back to metric depth [min_depth, max_depth]

    return o3d.geometry.Image(depth)

# Convert RGB and Depth Image to combined Point Cloud -------------------------------------------
def img_to_pcd(cams, dir='renders_scaled', min_depth=4.5, max_depth=5.5):
    
    pcds = []
    for i, cam in enumerate(cams):
        #extrinsics = np.linalg.inv(cam['ex_tensor'].numpy()) # using cam local
        extrinsics = cam['ex_tensor'].numpy() # using global system
        extrinsics = extrinsics.astype(np.float64)

        cam_pos = extrinsics[:3, 3].tolist()
        print(f"cam_{i+1}: {cam_pos}")

        intrinsics = cam['in_tensor'].numpy()
        intrinsics = intrinsics.astype(np.float64)

        cam_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            cam['w'], cam['h'], intrinsics
        )

        depth_img_path = os.path.join(dir, f"Camera_{i+1}/depth_png/depth_norm_0001.png")
        rgb_img_path = os.path.join(dir, f"Camera_{i+1}/rgb/rgb_0001.png")
            
        depth_img =load_normalized_depth(depth_img_path, min_depth, max_depth) # using opencv
        rgb_img = o3d.io.read_image(rgb_img_path)
        
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, depth_img, 1.0, max_depth, False) 
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam_intrinsics, extrinsics)

        pcds.append(pcd) 
    return pcds

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
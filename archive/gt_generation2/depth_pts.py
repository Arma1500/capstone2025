import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
import json
import cv2
import os

import polyscope as ps
ps.init()

def read_cam_data(path="../gt_RayCasting/camera_data_temp.json"):

    rot_y180_mat = np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rot_z90_mat = np.array([
        [0, -1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rot_zinv90_mat = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rot_x90_mat = np.array([
        [1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rot_yinv90_mat = np.array([
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rot_y90_mat = np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    cams = []
    with open(path) as f:
        cam_data = json.load(f)

        for i in range(4):
            cam_info = cam_data[i]
            
            #Get Matricies ------------------------------------------------------------------------------
            # Intrinsics
            in_mat = np.array(cam_info["intrinsics"]["intrinsic_matrix"], dtype=np.float32)

            # Extrinsics
            ex_mat = np.array(cam_info["extrinsic_mat"], dtype=np.float32)

            # if i == 1:
            #     ex_cam_rot_y180 = ex_mat @ rot_y180_mat
            #     ex_out = ex_cam_rot_y180

            # elif i == 2:
            #     ex_cam_rot_z90 = ex_mat @ rot_z90_mat

            #     ex_cam_rot_z90 = ex_cam_rot_z90 @ rot_yinv90_mat
            #     ex_out = ex_cam_rot_z90

            # elif i == 3:
            #     ex_cam_rot_zinv90 = ex_mat @ rot_zinv90_mat

            #     ex_cam_rot_zinv90 = ex_cam_rot_zinv90 @ rot_y90_mat
            #     ex_out = ex_cam_rot_zinv90

            # else:
            #     ex_out = ex_mat

            ex_out = ex_mat
            cam_pos = ex_out[:3, 3].tolist()
            print(f"cam_{i+1}: {cam_pos}")


            cam_tmp = {
                'ex_tensor' : o3d.core.Tensor(ex_out),
                'in_tensor' : o3d.core.Tensor(in_mat),
                'w' : cam_info["intrinsics"]["width"],
                'h' : cam_info["intrinsics"]["height"]
            }
            cams.append(cam_tmp)
    return cams

def img_to_pcd(cams, dir='../renders_for_test'):

    rot_y180_mat = np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rot_z90_mat = np.array([
        [0, -1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rot_zinv90_mat = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rot_yinv90_mat = np.array([
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rot_y90_mat = np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    
    pcds = []
    for i, cam in enumerate(cams):
        #extrinsics = np.linalg.inv(cam['ex_tensor'].numpy()) # using world?
        extrinsics = cam['ex_tensor'].numpy() # using cam local
        cam_extrinsics = extrinsics.astype(np.float64)

        intrinsics = cam['in_tensor'].numpy()
        intrinsics = intrinsics.astype(np.float64)

        cam_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            cam['w'], cam['h'], intrinsics
        )

        depth_img_path = os.path.join(dir, f"renders/Camera_{i+1}/depth_png/depth_norm_0001.png")
        rgb_img_path = os.path.join(dir, f"renders/Camera_{i+1}/rgb/rgb_0001.png")
            
        #depth_img =load_normalized_depth(depth_img_path) # using opencv
        depth_img = o3d.io.read_image(depth_img_path)
        rgb_img = o3d.io.read_image(rgb_img_path)
        
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, depth_img, 1.0, 5.0, False) 
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam_intrinsics)

        # if i == 1:
        #     ex_cam_rot_y180 = cam_extrinsics @ rot_y180_mat
        #     cam_extrinsics = ex_cam_rot_y180

        # elif i == 2:
        #     ex_cam_rot_z90 = cam_extrinsics @ rot_z90_mat

        #     ex_cam_rot_z90 = ex_cam_rot_z90 @ rot_yinv90_mat
        #     cam_extrinsics = ex_cam_rot_z90

        # elif i == 3:
        #     ex_cam_rot_zinv90 = cam_extrinsics @ rot_zinv90_mat

        #     ex_cam_rot_zinv90 = ex_cam_rot_zinv90 @ rot_y90_mat
        #     cam_extrinsics = ex_cam_rot_zinv90

        # else:
        #     cam_extrinsics = cam_extrinsics

        #pcd.transform(cam_extrinsics)
        
        pcds.append(pcd)
    return pcds

def polyscope_cam_params(camera_data): # This is just to visualise the cameras - they don't actually do anything
    #ex_world = np.linalg.inv(camera_data['ex_tensor'].numpy())
    ex_world = camera_data['ex_tensor'].numpy()
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

    # print(f"cam_pos: {cam_pos}")
    # print(f"look_dir: {look_dir}")
    print(f"up_dir: {up_dir}")

    return ps.CameraParameters(intrinsics, extrinsics)

ps.remove_all_structures()
cams_data = read_cam_data()
img_pcds = img_to_pcd(cams_data)

downsampled = img_pcds[0].voxel_down_sample(voxel_size=0.01)
points = np.asarray(downsampled.points)
depth_pcd = ps.register_point_cloud("Cam_1 points", points)

downsampled = img_pcds[1].voxel_down_sample(voxel_size=0.01)
points = np.asarray(downsampled.points)
depth_pcd = ps.register_point_cloud("Cam_2 points", points)

downsampled = img_pcds[2].voxel_down_sample(voxel_size=0.01)
points = np.asarray(downsampled.points)
depth_pcd = ps.register_point_cloud("Cam_3 points", points)

downsampled = img_pcds[3].voxel_down_sample(voxel_size=0.01)
points = np.asarray(downsampled.points)
depth_pcd = ps.register_point_cloud("Cam_4 points", points)

params = polyscope_cam_params(cams_data[0])
cam = ps.register_camera_view(f"Cam_1", params)

params = polyscope_cam_params(cams_data[1])
cam = ps.register_camera_view(f"Cam_2", params)

params = polyscope_cam_params(cams_data[2])
cam = ps.register_camera_view(f"Cam_3", params)

params = polyscope_cam_params(cams_data[3])
cam = ps.register_camera_view(f"Cam_4", params)

# depth_pcds = []
# for i, pcd in enumerate(img_pcds):
#     downsampled = pcd.voxel_down_sample(voxel_size=0.01)
#     points = np.asarray(downsampled.points) #downsampled.points)
#     if points.size > 0:
#         depth_pcd = ps.register_point_cloud(f"Camera {i+1} points", points)
#         depth_pcds.append(depth_pcd)

ps.show()
# for p in depth_pcds:
#     p.remove()


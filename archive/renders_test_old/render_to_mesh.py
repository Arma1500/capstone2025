import open3d as o3d
import cv2
import numpy as np
import matplotlib.pyplot as plt
import json
import os

import polyscope as ps
ps.init()

# FUNCTIONS -----------------------------------
# def look_at(eye, target, up):
#     forward = (target - eye)
#     forward /= np.linalg.norm(forward)

#     right = np.cross(up, forward)
#     right /= np.linalg.norm(right)

#     true_up = np.cross(forward, right)

#     R = np.stack([right, true_up, forward], axis=1)
#     T = eye.reshape(3, 1)

#     extrinsic = np.eye(4)
#     extrinsic[:3, :3] = R
#     extrinsic[:3, 3] = T[:, 0]

#     return extrinsic

# Load data from "camera_data.json"
def get_cam_data(directory):
    data = []

    with open(os.path.join(directory, "../gt_RayCasting/camera_data2.json")) as f:
        all_data = json.load(f)

    for i in range(4):
         cam_info = all_data[i]

         ex_mat = np.array(cam_info["extrinsic_mat"], dtype=np.float64) # maybe convert to trajectory later?
         ex_world = np.linalg.inv(ex_mat)

        #  R_blender_to_o3d_mesh = np.array([
        #      [1, 0, 0],
        #      [0, 0, -1],
        #      [0, 1, 0]
        #      ], dtype=np.float64)
        #  ex_world = ex_mat @ R_blender_to_o3d_mesh
        
        #print(f"ex_world: {ex_world}")
        #  cam_position = np.linalg.inv(ex_mat)[:3, 3]  # extract camera position in world coords
        #  # Recompute extrinsic: make camera look at origin
        #  target = np.array([0.0, 0.0, 0.0])
        #  up = np.array([0.0, 1.0, 0.0])  # or change to [0, 0, 1] if your cameras use Z-up
        #  new_ex = look_at(cam_position, target, up)

         in_mat = np.array(cam_info["intrinsics"]["intrinsic_matrix"], dtype=np.float64)
         
         intrinsic = o3d.camera.PinholeCameraIntrinsic(
              width=cam_info["intrinsics"]["width"],
              height=cam_info["intrinsics"]["height"],
              intrinsic_matrix=in_mat) 
         
         data.append({
            'extrinsic': ex_world,
            'intrinsic': intrinsic
          })
         
         print(f"cam {i}, ex_world: {ex_world}")


    return data

def load_normalized_depth(path):
    max_depth = 5.0
    depth = cv2.imread(path, cv2.IMREAD_UNCHANGED).astype(np.float32)
    depth = depth[:, :, 0] 
    depth = (depth / 255.0) * max_depth
    return o3d.geometry.Image(depth)


# MAIN ------------------------------------------------
if __name__=="__main__":
    dir = os.path.abspath("renders_for_test")
    cam_data = get_cam_data(dir)

    
    pcds = []
    frames = []
    for i in range(4):
        intrinsics = cam_data[i]['intrinsic']
        extrinsics = cam_data[i]['extrinsic']
        
        depth_img_path = os.path.join(dir, f"renders/Camera_{i+1}/depth_png/depth_norm_0001.png")
        rgb_img_path = os.path.join(dir, f"renders/Camera_{i+1}/rgb/rgb_0001.png")
        
        #depth_img =load_normalized_depth(depth_img_path) # using opencv
        depth_img = o3d.io.read_image(depth_img_path)
        rgb_img = o3d.io.read_image(rgb_img_path)
        
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, depth_img, 1.0, 3.0, False) 
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics, extrinsics)#, depth_trunc=3, stride=1)
        # this is without applying the camera extrinsics anywhere

        #print("point_clouds created")
        #R = np.linalg.inv(cam_data[0]['extrinsic'][:3, :3]) @ cam_data[i]['extrinsic'][:3, :3]
        #pcd.rotate(R)
        # flip_mat = np.eye(4)
        # flip_mat[2, 2] = -1
        # transform = cam_data[0]['extrinsic'] @ flip_mat
        #pcd = pcd.rotate(extrinsics[:3, :3])
        #pcd = pcd.translate((0, 0, 0))

        # flip_y = np.array([[1,  0,  0],
        #               [0, -1,  0],
        #               [0,  0, 1]], dtype=np.float32)
        # flip_y_4x4 = np.eye(4)
        # flip_y_4x4[:3, :3] = flip_y

        # theta = np.deg2rad(-45)
        # Rx = np.eye(4)
        # Rx[1, 1] = np.cos(theta)
        # Rx[1, 2] = -np.sin(theta)
        # Rx[2, 1] = np.sin(theta)
        # Rx[2, 2] = np.cos(theta)

        # transform = extrinsics @ flip_y_4x4

        # transform2 = transform @ Rx
        
        #pcd = pcd.transform(transform)
        #pcd = pcd.rotate(transform2[:3, :3])

        #pcd = pcd.transform(extrinsics)

        pcds.append(pcd)
        #downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.02)
        #o3d.visualization.draw_geometries([downsampled_pcd])
        #frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)#.transform(extrinsics)
        #o3d.visualization.draw_geometries([frame, downsampled_pcd])
        
        #frames.append(frame)
         
    # # Merge into one
    # merged_pcd = o3d.geometry.PointCloud()
    # for pcd in pcds:
    #     merged_pcd += pcd

    # # Visualise
    # downsampled_pcd = merged_pcd.voxel_down_sample(voxel_size=0.02)
    # o3d.visualization.draw_geometries(frames + [downsampled_pcd])

# for cam in cam_data:
#     frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
#     frame.transform(cam['extrinsic'])
#     o3d.visualization.draw_geometries([frame, *pcds])
# frames = [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(cam['extrinsic']) for cam in cam_data]
# o3d.visualization.draw_geometries(frames + [downsampled_pcd])

for i, pcd in enumerate(pcds):
    downsampled = pcd.voxel_down_sample(voxel_size=0.01)
    points = np.asarray(downsampled.points) #downsampled.points)
    if points.size > 0:
        ps.register_point_cloud(f"Camera {i+1} points", points)

ps.show()
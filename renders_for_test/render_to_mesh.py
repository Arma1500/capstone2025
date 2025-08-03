import open3d as o3d
import cv2
import numpy as np
import matplotlib.pyplot as plt
import json
import os

# FUNCTIONS -----------------------------------
# Load data from "camera_data.json"
def get_cam_data(directory):
    data = []

    with open(os.path.join(directory, "../gt_RayCasting/camera_data.json")) as f:
        all_data = json.load(f)

    for i in range(4):
         cam_info = all_data[i]

         ex_mat = np.array(cam_info["extrinsic_mat"], dtype=np.float64) # maybe convert to trajectory later?
         ex_world = np.linalg.inv(ex_mat)
         #print(f"ex_world: {ex_world}")

         in_mat = np.array(cam_info["intrinsics"]["intrinsic_matrix"], dtype=np.float64)
         
         intrinsic = o3d.camera.PinholeCameraIntrinsic(
              width=cam_info["intrinsics"]["width"],
              height=cam_info["intrinsics"]["height"],
              intrinsic_matrix=in_mat) 
         
         data.append({
            'extrinsic': ex_world,
            'intrinsic': intrinsic
          })

    return data

def load_normalized_depth(path):
    depth = cv2.imread(path, cv2.IMREAD_UNCHANGED).astype(np.float32)
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    depth = (depth / 255.0) * 3.0
    return o3d.geometry.Image(depth)


# MAIN ------------------------------------------------
if __name__=="__main__":
    dir = os.path.abspath("renders_for_test")
    cam_data = get_cam_data(dir)

    
    pcds = []
    for i in range(4):
        intrinsics = cam_data[i]['intrinsic']
        extrinsics = cam_data[i]['extrinsic']
        
        depth_img_path = os.path.join(dir, f"renders/Camera_{i+1}/depth_png/depth_norm_0001.png")
        rgb_img_path = os.path.join(dir, f"renders/Camera_{i+1}/rgb/rgb_0001.png")
        
        depth_img = load_normalized_depth(depth_img_path)
        rgb_img = o3d.io.read_image(rgb_img_path)
        
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, depth_img, 1.0, 3.0, False) 
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics, extrinsics)#, depth_trunc=3, stride=1)
        #print("point_clouds created")

        pcds.append(pcd)
         
    # Merge into one
    merged_pcd = o3d.geometry.PointCloud()
    for pcd in pcds:
        merged_pcd += pcd

    # Visualise
    downsampled_pcd = merged_pcd.voxel_down_sample(voxel_size=0.02)
    o3d.visualization.draw_geometries([downsampled_pcd])

# for cam in cam_data:
#     frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
#     frame.transform(cam['extrinsic'])
#     o3d.visualization.draw_geometries([frame, *pcds])

        

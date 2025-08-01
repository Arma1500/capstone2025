import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json
import os

# FUNCTIONS -----------------------------------
# Load data from "camera_data.json"
def get_cam_data(directory):
    data = []

    with open(os.path.join(directory, "../camera_data.json")) as f:
            all_data = json.load(f)

    for i in range(4):
         cam_info = all_data[i]

         in_mat = np.array(cam_info["intrinsics"]["intrinsic_matrix"], dtype=np.float64)
         print(in_mat)
         
         intrinsic = o3d.camera.PinholeCameraIntrinsic(
              width=cam_info["intrinsics"]["width"],
              height=cam_info["intrinsics"]["height"],
              intrinsic_matrix=in_mat) 
         
         data.append(intrinsic)

    return data

# MAIN ------------------------------------------------
if __name__=="__main__":
    dir = os.path.abspath("D:/Capstone/renders")

    cam_data = get_cam_data(dir)
    intrinsics = cam_data[1]
    print(intrinsics)

    depth_exr_path = os.path.join(dir, "Camera_1/depth_exr/depth_raw_0001.exr")
    depth_img = o3d.io.read_image(depth_exr_path)

    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_img, intrinsics)
    o3d.visualization.draw_geometries(pcd)

    # mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
    # o3d.visualization.draw_geometries([mesh])
    # o3d.io.write_triangle_mesh("output_mesh.ply", mesh)

        

import open3d as o3d
import opencv as cv2
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
         #print(in_mat)
         
         intrinsic = o3d.camera.PinholeCameraIntrinsic(
              width=cam_info["intrinsics"]["width"],
              height=cam_info["intrinsics"]["height"],
              intrinsic_matrix=in_mat) 
         
         data.append(intrinsic)

    return data

# MAIN ------------------------------------------------
if __name__=="__main__":
    dir = os.path.abspath("/media/humense/Expansion/Capstone/renders")

    cam_data = get_cam_data(dir)
    intrinsics = cam_data[1]

    depth_img_path = os.path.join(dir, "Camera_1/depth_png/depth_norm_0001.png")
    #depth_img = o3d.io.read_image(depth_img_path)
    depth_img = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED).astype(np.float32)
    depth_img = (depth_img/255.0)* 3.0
    depth_data = o3d.geometry.Image(depth_img)

    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_data, intrinsics)#, depth_trunc=3, stride=1)
    o3d.visualization.draw_geometries([pcd])

    # mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
    # o3d.visualization.draw_geometries([mesh])
    # o3d.io.write_triangle_mesh("output_mesh.ply", mesh)

        

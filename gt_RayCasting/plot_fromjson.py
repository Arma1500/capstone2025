import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json

def get_cams_extrinsics():
    cams = []
    with open("camera_new_data.json") as f:
        cam_data = json.load(f)

    for i in range(4):
        cam_info = cam_data[i]
        
        ex_mat = np.array(cam_info["ex_mat"], dtype=np.float64)
        #ex_world = np.linalg.inv(ex_mat)

        in_mat = np.array(cam_info["in_mat"], dtype=np.float64)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=cam_info["w"],
            height=cam_info["h"],
            intrinsic_matrix=in_mat
        )
        cams.append({
            'extrinsic': ex_mat,
            'intrinsic': intrinsic
        })

    return cams

def get_point_clouds():
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

    return point_clouds

if __name__=="__main__":

    pt_clds = get_point_clouds()

    # transform all clouds to where the first cloud is
    center = pt_clds[0].get_center()

    for i, pcd in enumerate(pt_clds):
        pcd.translate(center)

    merged_pcd = o3d.geometry.PointCloud()
    for pcd in pt_clds:
        merged_pcd += pcd

    # Visualize
    downsampled_pcd = merged_pcd.voxel_down_sample(voxel_size=0.02)
    o3d.visualization.draw_geometries([downsampled_pcd])

import open3d as o3d
import numpy as np
import json

import polyscope as ps
ps.init()

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

    for pcd in pt_clds:
        pcd.translate(center)

    merged_pcd = o3d.geometry.PointCloud()
    for pcd in pt_clds:
        merged_pcd += pcd

    # Visualise
    downsampled_pcd = merged_pcd.voxel_down_sample(voxel_size=0.02)
    #o3d.visualization.draw_geometries([downsampled_pcd])

    pt_cloud_vis = ps.register_point_cloud("hit data", np.asarray(downsampled_pcd.points))
    ps.show()


    # generate some points
    points = np.random.rand(100, 3)
    ps_cloud = ps.register_point_cloud("my points", points)
    ps.show()

    # ps_cloud_opt = ps.register_point_cloud("my points", points, radius=0.02, point_render_mode='quad')
    # ps.show()
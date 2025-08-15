import open3d as o3d
import numpy as np
import json

import polyscope as ps
ps.init()

def read_cam_data():
    cams = []
    with open("gt_RayCasting/camera_data4.json") as f:
        cam_data = json.load(f)

    for i in range(4):
        cam_info = cam_data[i]
        
        # Get Matricies
        ex_mat = np.array(cam_info["extrinsic_mat"], dtype=np.float32)
        cam_world = np.linalg.inv(ex_mat)
        cam_pos = cam_world[:3, 3]
        #cam_pos_world = cam_world[:3,3]
        #ex_mat_look = look_at(cam_pos_world, np.array(target_center)) # need to change mat to look at object
        
        in_mat = np.array(cam_info["intrinsics"]["intrinsic_matrix"], dtype=np.float32)

        cam_tmp = {
            'ex_mat' : cam_world,
            'pos' : cam_pos,
            'in_mat' : in_mat,
            'w' : cam_info["intrinsics"]["width"],
            'h' : cam_info["intrinsics"]["height"]
        }
        cams.append(cam_tmp)
    return cams

def get_point_clouds(cams):
    all_points = []
    all_directions = []

    for i in range(4):
        with open(f'gt_RayCasting/ground_truth/Camera_{i+1}/frame_0001.json', 'r') as f:
            camera_hit_list = json.load(f)

        # Extract just the hit points
        hit_points = np.array([entry['hit'] for entry in camera_hit_list if entry['hit'] is not None], dtype=np.float32)

        cam_origin = cams[i]['pos']
        direction = hit_points - cam_origin

        # Convert to Point Cloud
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(hit_points)

        all_points.append(hit_points)
        all_directions.append(direction)
        print("pt cld added")

    return all_points, all_directions 

if __name__=="__main__":

    # pt_clds = get_point_clouds()

    # # transform all clouds to where the first cloud is
    # center = pt_clds[0].get_center()

    # for pcd in pt_clds:
    #     pcd.translate(center)

    # merged_pcd = o3d.geometry.PointCloud()
    # for pcd in pt_clds:
    #     merged_pcd += pcd

    # # Visualise
    # downsampled_pcd = merged_pcd.voxel_down_sample(voxel_size=0.02)
    # #o3d.visualization.draw_geometries([downsampled_pcd])

    # pt_cloud_vis = ps.register_point_cloud("hit data", np.asarray(downsampled_pcd.points))
    # ps.show()


    # # generate some points
    # points = np.random.rand(100, 3)L
    # ps_cloud = ps.register_point_cloud("my points", points)
    # ps.show()

    # ps_cloud_opt = ps.register_point_cloud("my points", points, radius=0.02, point_render_mode='quad')
    # ps.show()

    cameras = read_cam_data()

    for i, cam in enumerate(cameras):
        cam_pts = cam['pos'].reshape(1,3)
        pc = ps.register_point_cloud(f"camera{i+1}", cam_pts)

        # Camera forward vector
        forward = -cam['ex_mat'][:3, 2].reshape(1,3)  # -Z in Blender
        pc.add_vector_quantity("directions", forward)


    hit_pts, directions = get_point_clouds(cameras)
    for i in range(len(hit_pts)):
        pcd = ps.register_point_cloud(f"Cam{i+1} hit points", hit_pts[i])
        #pcd.add_vector_quantity(f"Cam{i+1} vectors", directions[i], enabled=True)

    ps.show()

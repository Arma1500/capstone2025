import open3d as o3d
import numpy as np
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
def img_to_pcd(cams, dir, frame_name, min_depth, max_depth):
    
    pcds = []
    for i, cam in enumerate(cams):
        # get camera intrinsics and extrinsics
        extrinsics = cam['ex_tensor'].numpy() # using global system
        extrinsics = extrinsics.astype(np.float64)

        intrinsics = cam['in_tensor'].numpy()
        intrinsics = intrinsics.astype(np.float64)

        cam_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            cam['w'], cam['h'], intrinsics
        )

        # access depth image
        depth_img_path = os.path.join(dir, f"Camera_{i+1}/depth_png/depth_norm_{frame_name}.png")
        rgb_img_path = os.path.join(dir, f"Camera_{i+1}/rgb/rgb_{frame_name}.png")
            
        depth_img =load_normalized_depth(depth_img_path, min_depth, max_depth) # using opencv
        rgb_img = o3d.io.read_image(rgb_img_path)
        
        # create point cloud
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, depth_img, 1.0, max_depth, False) 
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam_intrinsics, extrinsics)

        # estimate normals
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
        pcd.orient_normals_towards_camera_location(extrinsics[:3, 3].tolist()) # orient based on the camera position?

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

    return ps.CameraParameters(intrinsics, extrinsics)


# MAIN FUNCTION
if __name__=="__main__":

    # File Path Set Up -------------------------------------------------------------------------
    camera_data_path = 'blender_camera_data.json'
    renders_path = '../simulation/pcds'

    output_path = '../simulation/pcds_normals'

    # Point Cloud Generation --------------------------------------------------------------------
    cams_data = read_cam_data(camera_data_path)

    # same as the scale used for the depth renders from blender scene
    min_depth=3.8
    max_depth=6.8

    start_frame = 1
    end_frame = 1
    frames = [str(i).zfill(4) for i in range(start_frame, end_frame + 1)]
    
    for frame in frames:
        print(f"Processing frame: {frame}")
        pcds = img_to_pcd(cams_data, renders_path, frame, min_depth, max_depth)

        # Merge the point clouds from all cameras
        merged_pcd = o3d.geometry.PointCloud()
        for pcd in pcds:
            merged_pcd += pcd

        # # convert normals to float to use in Meshlab
        # if merged_pcd.has_normals():
        #     merged_pcd.normals = o3d.utility.Vector3dVector(
        #         np.asarray(merged_pcd.normals, dtype=np.float32)
        #     )

        # Save the merged point cloud
        output_filename = os.path.join(output_path, f"render_frame_{frame}.ply")
        o3d.io.write_point_cloud(output_filename, merged_pcd)
        print(f"Saved point cloud to: {output_filename}")

    # display for debugging ----------------------------------------------
    # display cameras in polyscope scene
    for i, cam in enumerate(cams_data):
        print(f"Camera_{i+1} --------------------------")
        params = polyscope_cam_params(cam)
        cam = ps.register_camera_view(f"Camera {i+1}", params)

    # displaying the last pointcloud
    for i, pcd in enumerate(pcds):
        downsampled = pcd.voxel_down_sample(voxel_size=0.01)
        points = np.asarray(downsampled.points) #downsampled.points)
        normals = np.asarray(downsampled.normals)
        if points.size > 0:
            depth_pcd = ps.register_point_cloud(f"Camera {i+1} points", points)
            depth_pcd.add_vector_quantity("normals", normals, enabled=True)

    ps.show()
    ps.remove_all_structures()
    # ----------------------------------------------------------------------
    
    print("Has normals:", merged_pcd.has_normals())

    
#### PROBLEM: She is moving towards one camera and away from the other so the front and back cameras don't capture the depth
#### properly. I have to adjust the scale later

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json
import ast

def build_dict(ans, object):

    for idx, a in enumerate(ans):
        width_px = 1920
        height_px = 1080
        primitive_ids = a["primitive_ids"].numpy().reshape((height_px, width_px ))
        primitive_uvs = a["primitive_uvs"].numpy().reshape((height_px, width_px , 2))
        
        triangle_ids = np.asarray(object.triangles)
        vertices = np.asarray(object.vertices)
        triangle_vertices = vertices[triangle_ids]
        
        hit_data = {
            f"camera_{idx}": {
                "ray_hits": []
            }
        }

        for y in range(height_px):
            for x in range(width_px):
                tri_id = primitive_ids[y,x]
                
                if tri_id == o3d.t.geometry.RaycastingScene.INVALID_ID:
                    continue  # Skip rays that didn't hit anything

                u, v = primitive_uvs[y, x]
                v0, v1, v2 = triangle_vertices[tri_id]
                hit_point = (1 - u - v) * v0 + u * v1 + v * v2

                dv1 = float(np.linalg.norm(hit_point - v0))
                dv2 = float(np.linalg.norm(hit_point - v1))
                dv3 = float(np.linalg.norm(hit_point - v2))

                hit_data[f"camera_{idx}"]["ray_hits"].append({
                    "pixel": [x, y],
                    "triangle_id": int(tri_id), #oof I feel a bit iffy about this one
                    "hit": hit_point.tolist(),
                    "dv1": dv1,
                    "dv2": dv2,
                    "dv3": dv3
                })

    print("dictionary created!!!!1")

    return hit_data

def get_intrinsic_matrix():
    # camera intrinsics the same for all cams
    focal_length = 50
    sensor_width = 36
    width_px = 1920 
    height_px = 1080
    
    fx = fy = focal_length * (width_px / sensor_width)
    cx = width_px / 2
    cy = height_px / 2

    K = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float32)

    return o3d.core.Tensor(K, dtype=o3d.core.Dtype.Float32)

def get_extrinsic_matrix(cam_data):
    # Convert Blender camera orientation to Open3D
    R_blender = np.array(cam_data["rotation_matrix"], dtype=np.float32)
    t = np.array(cam_data["position"], dtype=np.float32).reshape(3, 1)

    # Fix coordinate system (Blender to Open3D)
    blender_to_open3d = np.array([
        [1, 0,  0],
        [0, 0,  1],
        [0, -1, 0]
    ], dtype=np.float32)

    R = blender_to_open3d @ R_blender
    t = blender_to_open3d @ t

    cam_to_world = np.eye(4, dtype=np.float32)
    cam_to_world[:3, :3] = R
    cam_to_world[:3, 3] = t[:, 0]

    world_to_cam = np.linalg.inv(cam_to_world)

    return o3d.core.Tensor(world_to_cam, dtype=o3d.core.Dtype.Float32)

def create_cameras():
    cams = []

    intrinsics = get_intrinsic_matrix()
    
    # getting extrinsics from blender and putting them together
    with open("test/sim_camera_extrinsics.txt", "r") as f:
        for line in f:
            cam_data = ast.literal_eval(line.strip())  # Convert string to dictionary
            extrinsics = get_extrinsic_matrix(cam_data)

            cam = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
                intrinsic_matrix=intrinsics,
                extrinsic_matrix=extrinsics,
                width_px=1920,
                height_px=1080
                )
            cams.append(cam)

    return cams

def ray_cast(object, cams):

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(object))

    ans = []
    for cam in cams:
        ans.append(scene.cast_rays(cam))
    
    return ans

if __name__=="__main__":

    line = "{'position': [0.0, 0.0, 0.0], " \
    "'rotation_matrix': [[0.009999999776482582, 0.0, 0.0], [0.0, 7.549789682315122e-10, -0.009999999776482582], [0.0, 0.009999999776482582, 7.549789682315122e-10]]}"
    data = ast.literal_eval(line)
    R = np.array(data['rotation_matrix'], dtype=np.float32)
    t = np.array(data['position'], dtype=np.float32)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = t

    T_inv = np.linalg.inv(T)

    object = o3d.io.read_triangle_mesh("test/frame_0001.ply")
    object.transform(T_inv)

    cams = create_cameras()
    ans = ray_cast(object, cams)

    for a in ans:
        plt.imshow(a['t_hit'].numpy())
        plt.show()

    #all_hit_data = build_dict(1, ans, object)
    #print("Dictionary Created! ___________________________________________________________")

    #with open(f"ground_truth{1}.json", "w") as f: # change to add loop when doing all of the objects
    #   json.dump(all_hit_data, f, indent=2)


    ## might be good to use if I move over to baba's laptop tomorrow:
    # o3d.visualization.draw_geometries([your_mesh, your_camera_frame])
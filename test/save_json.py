import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json

def build_dict(camera_number, ans, object):

    width_px = 1920
    height_px = 1080
    primitive_ids = ans["primitive_ids"].numpy().reshape((height_px, width_px ))
    primitive_uvs = ans["primitive_uvs"].numpy().reshape((height_px, width_px , 2))
    
    triangle_ids = np.asarray(object.triangles)
    vertices = np.asarray(object.vertices)
    triangle_vertices = vertices[triangle_ids]
    
    camera_data = {
        f"camera_{camera_number}": {
#            "intrinsics": intrinsics,
#            "extrinsics": extrinsics,
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

            camera_data[f"camera_{camera_number}"]["ray_hits"].append({
                "pixel": [x, y],
                "triangle_id": int(tri_id), #oof I feel a bit iffy about this one
                "hit": hit_point.tolist(),
                "dv1": dv1,
                "dv2": dv2,
                "dv3": dv3
            })

    print("dictionary created!!!!1")

    return camera_data

def ray_cast(object):

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(object))

    # Set up cameras
    cam = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        fov_deg=90,
        center=[0, 0, 1],
        eye=[2, 0, 0],
        up=[0, 1, 0],
        width_px=1920,
        height_px=1080,
    )
    ans = scene.cast_rays(cam)
    return ans

if __name__=="__main__":

    # load object
    object = mesh = o3d.io.read_triangle_mesh("frame_0001.ply")
    ans = ray_cast(object)
    print("ray_tracing done!")

    # plt.imshow(ans['t_hit'].numpy())
    # plt.show()

    all_camera_data = build_dict(1, ans, object) # change to add camera data later
    print("Dictionary Created! ___________________________________________________________")

    with open(f"ground_truth{1}.json", "w") as f: # change to add loop when doing all of the objects
       json.dump(all_camera_data, f, indent=2)
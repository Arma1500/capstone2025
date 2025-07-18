import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def ray_cast(object):

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(object))

    # Set up cameras
    cam1 = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        fov_deg=90,
        center=[0, 0, 1],
        eye=[2, 0, 0],
        up=[0, 1, 0],
        width_px=1920,
        height_px=1080,
    )
    ans = scene.cast_rays(cam1)

    return ans

def save_data(ans, object, num):

    width_px = 1920
    height_px = 1080

    primitive_ids = ans["primitive_ids"].numpy().reshape((height_px, width_px ))
    primitive_uvs = ans["primitive_uvs"].numpy().reshape((height_px, width_px , 2))

    triangle_ids = np.asarray(object.triangles)
    vertices = np.asarray(object.vertices)
    triangle_vertices = vertices[triangle_ids]  # shape: (N_triangles, 3, 3)

    count = 1
    
    with open(f"gt_{num}.txt", "w") as f:
        #f.write("x,y,triangle_id,hit_x,hit_y,hit_z,dv1,dv2,dv3\n")
        for y in range(height_px):
            for x in range(width_px):
                tri_id = primitive_ids[y,x]
                if tri_id == o3d.t.geometry.RaycastingScene.INVALID_ID:
                    continue  # Skip rays that didn't hit anything
                else:

                    count += 1

                    # Get triangle vertices
                    v0, v1, v2 = triangle_vertices[tri_id]

                    # Compute barycentric coords
                    u, v = primitive_uvs[y, x]
                    hit_point = (1 - u - v) * v0 + u * v1 + v * v2

                    # Distances to triangle vertices
                    dv1 = np.linalg.norm(hit_point - v0)
                    dv2 = np.linalg.norm(hit_point - v1)
                    dv3 = np.linalg.norm(hit_point - v2)

                    # Write line
                    f.write(f"{x},{y},{tri_id},{hit_point[0]:.6f},{hit_point[1]:.6f},{hit_point[2]:.6f},{dv1:.6f},{dv2:.6f},{dv3:.6f}\n")
    print(f"Hit Count: {count}")


if __name__=="__main__":
    
    #object = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
    #object = o3d.geometry.TriangleMesh.create_torus().translate([0, 0, 2])
    object = mesh = o3d.io.read_triangle_mesh("frame_0001.ply")

    #print('Vertices:')
    #print(len(np.asarray(mesh.vertices)))
    #print('Triangles:')
    #print((len(np.asarray(mesh.triangles))))
    
    ans = ray_cast(object)
    save_data(ans,object, 1)

    with open("gt_1.txt", "r") as f:
        num_lines = sum(1 for _ in f)
    print(f"The number of lines in the file: {num_lines}")

    plt.imshow(ans['t_hit'].numpy())
    plt.show()
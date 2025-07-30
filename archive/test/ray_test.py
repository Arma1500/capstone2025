import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def ray_cast(object):

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(object))

    # Set up cameras
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
    fov_deg=90,
    center=[0, 0, 2],
    eye=[2, 3, 0],
    up=[0, 1, 0],
    width_px=640,
    height_px=480,
    )
    # We can directly pass the rays tensor to the cast_rays function.
    ans = scene.cast_rays(rays)
    
    # constructing a point cloud from it
    hit = ans['t_hit'].isfinite()
    points = rays[hit][:,:3] + rays[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))
    pcd = o3d.t.geometry.PointCloud(points)
    # Press Ctrl/Cmd-C in the visualization window to copy the current viewpoint
    #o3d.visualization.draw_geometries([pcd.to_legacy()],
    #                                  front=[0.5, 0.86, 0.125],
    #                                  lookat=[0.23, 0.5, 2],
    #                                  up=[-0.63, 0.45, -0.63],
    #                                  zoom=0.7)
    # o3d.visualization.draw([pcd]) # new API

    
    o3d.visualization.draw_geometries([object, rays])

    return ans

if __name__=="__main__":
    
    #object = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
    object = o3d.geometry.TriangleMesh.create_torus().translate([0, 0, 2])
    #object = o3d.io.read_triangle_mesh("frame_0001.ply")

    #print('Vertices:')
    #print(len(np.asarray(mesh.vertices)))
    #print('Triangles:')
    #print((len(np.asarray(mesh.triangles))))
    
    ans = ray_cast(object)

    plt.imshow(ans['t_hit'].numpy())
    plt.show()
    
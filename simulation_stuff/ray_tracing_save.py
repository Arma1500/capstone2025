import bpy
import os
import mathutils
import json
import bpy_extras
from bpy_extras import view3d_utils

# Function to render the scene
def render_scene():
    bpy.context.scene.render.engine = 'Eevee'
    bpy.context.scene.frame_end = 10  # Adjust based on animation length
    bpy.ops.render.render(animation=True)
    
    
# Camera data  ___________________________________________________  
def get_camera_extrinsics(camera):
    location = list(camera.matrix_world.translation)
    rotation = camera.matrix_world.to_3x3()
    rotation_matrix = [list(row) for row in rotation]
    return {
        "position": location,
        "rotation_matrix": rotation_matrix
    }
    
def get_camera_intrinsics(camera):
    cam_data = camera.data
    return {
        "focal_length_mm": cam_data.lens,
        "resolution_px": (1920, 1080),
        "frame_rate": bpy.context.scene.render.fps
    }
    
# Ray Tracing  
def ray_trace(camera, scene, charecter, resolution=(1920, 1080)):
    w, h = resolution
    ray_data = []
    
    # realated to the geometry nodes I think - I might look into it later as well
    depsgraph = bpy.context.evaluated_depsgraph_get()
    evaluated_obj = charecter.evaluated_get(depsgraph)
    mesh = evaluated_obj.to_mesh()
    
    for y in range(h):
        for x in range(w):
            u = x / w
            v = y / h

            # Convert 2D normalized coords to 3D ray
            ray_origin = camera.matrix_world.translation
            ray_target = bpy_extras.object_utils.world_to_camera_view(scene, camera, ray_origin)
            ray_direction = (camera.matrix_world @ mathutils.Vector((0.0, 0.0, -1.0)) - ray_origin).normalized()

            result, location, normal, face_index, *_, _ = scene.ray_cast(depsgraph, ray_origin, ray_direction) # so the ray cast function is from the geometry nodes?

            if result and face_index is not None:
                triangle = mesh.polygons[face_index]
                verts = [mesh.vertices[i].co for i in triangle.vertices]
                dv1 = (location - verts[0]).length
                dv2 = (location - verts[1]).length
                dv3 = (location - verts[2]).length

                ray_data.append({
                    "pixel": [x, y],
                    "triangle_id": face_index,
                    "hit": list(location),
                    "dv1": dv1,
                    "dv2": dv2,
                    "dv3": dv3
                })

    evaluated_obj.to_mesh_clear()
    return ray_data
    

# Main execution ________________________________________________________________________

# Set up frames
start_frame = bpy.context.scene.frame_start
end_frame = 2 #bpy.context.scene.frame_end
total_frames = end_frame - start_frame + 1
frame_index = 0

# Set up scene
scene = bpy.context.scene
camera_names = ['Camera_1', 'Camera_2', 'Camera_3', 'Camera_4']
character = bpy.data.objects.get('body')
print("Set scene: Finished")

# Ray Tracing from each camera and Saving to .json file
output_base_path = "C:/Work/Capstone/ground_truth"  # Change to your preferred path
os.makedirs(output_base_path, exist_ok=True)
output_json_path = os.path.join(output_base_path, f"gt{1}.json")

for frame in range(start_frame, end_frame + 1):
    scene.frame_set(frame)
    print(f"\n🕘 Frame {frame}:")
    
    output_json_path = os.path.join(output_base_path, f"gt{frame}.json")  

    result = {}
    for cam_name in camera_names:
        
        cam = bpy.data.objects.get(cam_name)
        if cam is None:
            print(f"❌ Camera '{cam_name}' not found!")
            continue

        # camera info
        intrinsics = get_camera_intrinsics(cam)
        extrinsics = get_camera_extrinsics(cam)
        
        # performing ray tracing
        print(f"Ray tracing from {cam_name}...")
        ray_hits = ray_trace(cam, scene, character)
        
        result[cam_name.lower()] = {
            "intrinsics": intrinsics,
            "extrinsics": extrinsics,
            "ray_hits": ray_hits
        }
        
        with open(output_json_path, "w") as f:
            json.dump(result, f, indent=4)
            
        frame_index += 1

print(f"✅ Ray tracing data saved to {output_json_path}")
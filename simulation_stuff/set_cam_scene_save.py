import bpy
import os
import mathutils
import numpy as np
import json
    

def get_camera_extrinsics(cam_obj):
    # World to camera transform
    world_to_cam = np.array(cam_obj.matrix_world.inverted())

    # Blender camera coordinates → OpenCV convention
    # Blender: +X right, +Y up, -Z forward
    # OpenCV : +X right, -Y down, +Z forward
    R_bcam2cv = np.array([[1,  0,  0],
                          [0, -1,  0],
                          [0,  0, -1]], dtype=np.float32)

    # Apply rotation fix
    Rt = world_to_cam.copy()
    Rt[:3, :3] = R_bcam2cv @ Rt[:3, :3]
    
    print(Rt)

    return Rt.tolist()

#def get_camera_extrinsics(camera):
##    location = list(camera.matrix_world.translation)
##    rotation = camera.matrix_world.to_3x3()
##    rotation_matrix = [list(row) for row in rotation]
##    
##    extrinsics = {
##        "position": location,
##        "rotation_matrix": rotation_matrix
##        }
#        
##    mat = camera.matrix_world # needs to be inverted for using otside of blender
##    extrinsic_mat = np.array(mat, dtype=np.float32)
#    
#    blender_cam_to_world = np.array(camera.matrix_world, dtype=np.float32)
#    R_blender_to_o3d_coords = np.array([[1, 0, 0, 0],
#    [0, 0, 1, 0],  # Blender's Z (up) becomes Open3D's Y (up)
#    [0, -1, 0, 0], # Blender's Y (depth) becomes Open3D's -Z (depth)
#    [0, 0, 0, 1]], dtype=np.float32)
#    o3d_cam_to_world = blender_cam_to_world @ R_blender_to_o3d_coords
#    extrinsic_mat = o3d_cam_to_world #np.linalg.inv(o3d_cam_to_world)
#    
#    print(f"extrinsic_mat: {extrinsic_mat}")
#    return extrinsic_mat
    
def get_camera_intrinsics(camera_obj):
    render = bpy.context.scene.render
    cam_data = camera_obj.data

    # Get resolution
    resolution_x = render.resolution_x * render.resolution_percentage / 100
    resolution_y = render.resolution_y * render.resolution_percentage / 100
    
    #print(f"check resolution: {resolution_x}, {resolution_y}")

    # Sensor fit: horizontal or vertical
    sensor_fit = cam_data.sensor_fit
    if sensor_fit == 'AUTO':
        sensor_fit = 'HORIZONTAL' if resolution_x >= resolution_y else 'VERTICAL'

    if sensor_fit == 'HORIZONTAL':
        fx = (resolution_x * cam_data.lens) / cam_data.sensor_width
        fy = fx
    else:
        fy = (resolution_y * cam_data.lens) / cam_data.sensor_height
        fx = fy

    # Focal Point
    cx = resolution_x / 2.0
    cy = resolution_y / 2.0

    # Create Matrix
    intrinsic_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0,  0,  1]
    ], dtype=np.float32)

    # Save to Dict
    intrinsics = {
        "width": int(resolution_x),
        "height": int(resolution_y),
        "intrinsic_matrix": intrinsic_matrix.tolist()
    }

    return intrinsics

# Function to add cameras
def add_cameras():
    distance = 5  # Distance from the center
    camera_positions = [
        (distance, 0, 2),  # Right
        (-distance, 0, 2), # Left
        (0, distance, 2),  # Front
        (0, -distance, 2)  # Back
    ]
    
    cameras = []
    
    for i, pos in enumerate(camera_positions):
        cam = bpy.data.cameras.new(name=f"Camera_{i+1}")
        cam_obj = bpy.data.objects.new(name=f"Camera_{i+1}", object_data=cam)
        bpy.context.collection.objects.link(cam_obj)
        
        cam_obj.location = pos
        #cam_obj.rotation_euler = bpy.context.scene.cursor.location.to_3d().rotation_difference(cam_obj.location).to_euler()
        
        # Aim camera at the origin (0, 0, 0)
        direction = mathutils.Vector((0, 0, 0)) - cam_obj.location
        rot_quat = direction.to_track_quat('-Z', 'Y')  # Camera looks down -Z by default
        cam_obj.rotation_euler = rot_quat.to_euler()
        
        cameras.append(cam_obj)
    
    return cameras

# Function to import a 3D dancing character
def import_dancing_character(path):
    if not os.path.exists(path):
        print(f"Error: File {path} not found!")
        return None

    bpy.ops.import_mesh.ply(filepath=path)
    
    # Assuming the first imported object is the character
    character = bpy.context.selected_objects[0]
    character.location = (0, 0, 0)  # Center it
    character.scale = (0.01, 0.01, 0.01)  # Adjust scale if needed
    character.rotation_euler = (1.57, 0, 0)

    print(f"Dancing character imported: {character.name}")
    return character

# Function to clear existing cameras and characters
def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
# Function to render the scene
def render_scene():
    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.frame_end = 100  # Adjust based on animation length
    bpy.ops.render.render(animation=True)

# Main execution
clear_scene()
cameras = add_cameras()

# Force Blender to update transforms
bpy.context.view_layer.update()

output_base_path = "C:/Work/Capstone/"  # Change to your preferred path
output_path_cam = os.path.join(output_base_path, "camera_data2.json")
data_list = []

for cam in cameras:
    cam_name = cam.name
    if cam is None:
        print(f"❌ Camera '{cam_name}' not found!")
        continue

    camera_extrinsics = get_camera_extrinsics(cam)
    camera_intrinsics = get_camera_intrinsics(cam)
    
    data = {
        "extrinsic_mat" : camera_extrinsics,
        "intrinsics" : camera_intrinsics
        }
    data_list.append(data)
    print(f"Camera Data added for {cam_name}")
        
with open(output_path_cam, "w") as f:
    json.dump(data_list, f, indent=4)
            
print("ALL FILES SAVED")

# Set the path to your FBX character file (download from Mixamo or other sources)
#character_path = "C:/Work/Capstone/ground_truth/mesh_dancer/frame_0001.ply"  # <-- Change this to your file
#character = import_dancing_character(character_path)

#if character:
#    render_scene()

# to do:l
# set up ray caster
# set up dictionary to save the data to
# loop through 2 meshes to test
# send to other laptop and test on all meshes
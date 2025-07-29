import bpy
import os

# SAVING THE MODEL PLY FOR EACH FRAME FOR RAY-TRACING
output_mesh_path = "/home/humense/arhma-capstone/simulation/ground_truth/mesh_dancer"  # Change to your desired output directory
os.makedirs(output_mesh_path, exist_ok=True)

scene = bpy.context.scene
total_frames = scene.frame_end - scene.frame_start + 1
frame_index = 0

print("STARTING SAVE")
for frame in range(scene.frame_start, scene.frame_end + 1):
    print(f"Saving Mesh in frame: {frame} ...")
    scene.frame_set(frame)
    
    obj = bpy.data.objects.get('body')
    if obj is None:
        print(f"❌ Character '{character_name}' not found!")
        continue
    # Make sure the object is selected and active
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    
    # Apply modifiers (especially armature deform) as mesh for export
    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh().copy()  # Make a copy to use in main database
    temp_mesh = bpy.data.objects.new(f"{obj.name}_Export", mesh)
#    temp_mesh.location = (0, 0, 0)  # Center it
    temp_mesh.scale = (0.01, 0.01, 0.01)  # Adjust scale if needed
#    temp_mesh.rotation_euler = (1.57, 0, 0)
    bpy.context.collection.objects.link(temp_mesh)
    
    # Export to .PLY
    filepath = os.path.join(output_mesh_path, f"frame_{frame:04d}.ply")
    bpy.ops.object.select_all(action='DESELECT')
    temp_mesh.select_set(True)
    bpy.context.view_layer.objects.active = temp_mesh
    
    bpy.ops.wm.ply_export(
        filepath=filepath,
        export_selected_objects=True,
        #export_uv=True,
        export_normals=True,
        export_triangulated_mesh =True,
    )
    print(f"✅ Exported mesh for frame {frame} to {filepath}")
    
    # Clean up temporary mesh object
    bpy.data.objects.remove(temp_mesh)
    bpy.data.meshes.remove(mesh)  # Clean up the copied mesh
    
    frame_index += 1

print("\n✅ Ground truth meshes exported successfully.")

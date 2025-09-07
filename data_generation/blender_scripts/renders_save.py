import bpy
import os

# === CONFIGURATION ===
camera_names = ['Camera_1', 'Camera_2', 'Camera_3', 'Camera_4']
output_base_path = "/home/humense/arhma-capstone/simulation/renders2"  # Change to your preferred path
scene = bpy.context.scene
start_frame = bpy.context.scene.frame_start
end_frame = 2 #bpy.context.scene.frame_end

# === COMPOSITOR SETUP FUNCTION ===
def setup_compositor_output(cam_name):
    scene.use_nodes = True
    tree = scene.node_tree
    tree.nodes.clear()

    rl = tree.nodes.new('CompositorNodeRLayers')

    # Camera-specific base path
    cam_output_dir = os.path.join(output_base_path, cam_name)

    # RGB output
    rgb_output = tree.nodes.new(type="CompositorNodeOutputFile")
    rgb_output.label = "RGB Output"
    rgb_output.base_path = os.path.join(cam_output_dir, "rgb")
    rgb_output.format.file_format = 'PNG'
    rgb_output.format.color_mode = 'RGB'
    rgb_output.format.color_depth = '8'
    rgb_output.file_slots[0].path = "rgb_####"

    # Normalized depth (8-bit PNG)
    map_range = tree.nodes.new('CompositorNodeMapRange')
    map_range.inputs['From Min'].default_value = 4.5
    map_range.inputs['From Max'].default_value = 5.5
    map_range.inputs['To Min'].default_value = 0
    map_range.inputs['To Max'].default_value = 1

    norm_depth_output = tree.nodes.new(type="CompositorNodeOutputFile")
    norm_depth_output.label = "Normalized Depth PNG"
    norm_depth_output.base_path = os.path.join(cam_output_dir, "depth_png")
    norm_depth_output.format.file_format = 'PNG'
    norm_depth_output.format.color_mode = 'RGB'
    norm_depth_output.format.color_depth = '8'
    norm_depth_output.file_slots[0].path = "depth_norm_####"

#    # Raw depth (32-bit OpenEXR)
#    raw_depth_output = tree.nodes.new(type="CompositorNodeOutputFile")
#    raw_depth_output.label = "Raw Depth EXR"
#    raw_depth_output.base_path = os.path.join(cam_output_dir, "depth_exr")
#    raw_depth_output.format.file_format = 'OPEN_EXR'
#    raw_depth_output.format.color_mode = 'RGB'
#    raw_depth_output.format.color_depth = '32'
#    raw_depth_output.file_slots[0].path = "depth_raw_####"

    # Connect nodes
    tree.links.new(rl.outputs['Image'], rgb_output.inputs[0])
    tree.links.new(rl.outputs['Depth'], map_range.inputs[0])
    tree.links.new(map_range.outputs[0], norm_depth_output.inputs[0])
#    tree.links.new(rl.outputs['Depth'], raw_depth_output.inputs[0])

# === RENDER LOOP ===
total_frames = end_frame - start_frame + 1
frame_index = 0

print("🎬 Starting rendering...")

for frame in range(start_frame, end_frame + 1):
    scene.frame_set(frame)
    print(f"\n🕘 Frame {frame}:")

    for cam_name in camera_names:
        cam = bpy.data.objects.get(cam_name)
        if cam is None:
            print(f"❌ Camera '{cam_name}' not found!")
            continue

        scene.camera = cam
        setup_compositor_output(cam_name)
        bpy.ops.render.render(write_still=True)
        print(f"   ✅ Rendered {cam_name}")

    frame_index += 1
    percent_done = (frame_index / total_frames) * 100
    print(f"📈 Progress: {percent_done:.1f}%")

print("\n✅ All frames and all cameras rendered successfully.")
import bpy
import os

# === CONFIGURATION ===
camera_names = ['Camera_1', 'Camera_2', 'Camera_3', 'Camera_4']
output_base_path = "/home/humense/arhma-capstone/simulation/renders2"  # Change to your preferred path
scene = bpy.context.scene
start_frame = bpy.context.scene.frame_start
end_frame = bpy.context.scene.frame_end

# === COMPOSITOR SETUP FUNCTION ===
def setup_compositor_output(cam_name):
    scene.use_nodes = True
    tree = scene.node_tree
    tree.nodes.clear()

    rl = tree.nodes.new('CompositorNodeRLayers')

    # Camera-specific base path
    cam_output_dir = os.path.join(output_base_path, cam_name)

    # RGB output
    rgb_output = tree.nodes.new(type="CompositorNodeOutputFile")
    rgb_output.label = "RGB Output"
    rgb_output.base_path = os.path.join(cam_output_dir, "rgb")
    rgb_output.format.file_format = 'PNG'
    rgb_output.format.color_mode = 'RGB'
    rgb_output.format.color_depth = '8'
    rgb_output.file_slots[0].path = "rgb_####"

    # Normalized depth (8-bit PNG)
    map_range = tree.nodes.new('CompositorNodeMapRange')
    map_range.inputs['From Min'].default_value = 4.5
    map_range.inputs['From Max'].default_value = 5.5
    map_range.inputs['To Min'].default_value = 0
    map_range.inputs['To Max'].default_value = 1

    norm_depth_output = tree.nodes.new(type="CompositorNodeOutputFile")
    norm_depth_output.label = "Normalized Depth PNG"
    norm_depth_output.base_path = os.path.join(cam_output_dir, "depth_png")
    norm_depth_output.format.file_format = 'PNG'
    norm_depth_output.format.color_mode = 'RGB'
    norm_depth_output.format.color_depth = '8'
    norm_depth_output.file_slots[0].path = "depth_norm_####"

#    # Raw depth (32-bit OpenEXR)
#    raw_depth_output = tree.nodes.new(type="CompositorNodeOutputFile")
#    raw_depth_output.label = "Raw Depth EXR"
#    raw_depth_output.base_path = os.path.join(cam_output_dir, "depth_exr")
#    raw_depth_output.format.file_format = 'OPEN_EXR'
#    raw_depth_output.format.color_mode = 'RGB'
#    raw_depth_output.format.color_depth = '32'
#    raw_depth_output.file_slots[0].path = "depth_raw_####"

    # Connect nodes
    tree.links.new(rl.outputs['Image'], rgb_output.inputs[0])
    tree.links.new(rl.outputs['Depth'], map_range.inputs[0])
    tree.links.new(map_range.outputs[0], norm_depth_output.inputs[0])
#    tree.links.new(rl.outputs['Depth'], raw_depth_output.inputs[0])

# === RENDER LOOP ===
total_frames = end_frame - start_frame + 1
frame_index = 0

print("🎬 Starting rendering...")

for frame in range(start_frame, end_frame + 1):
    scene.frame_set(frame)
    print(f"\n🕘 Frame {frame}:")

    for cam_name in camera_names:
        cam = bpy.data.objects.get(cam_name)
        if cam is None:
            print(f"❌ Camera '{cam_name}' not found!")
            continue

        scene.camera = cam
        setup_compositor_output(cam_name)
        bpy.ops.render.render(write_still=True)
        print(f"   ✅ Rendered {cam_name}")

    frame_index += 1
    percent_done = (frame_index / total_frames) * 100
    print(f"📈 Progress: {percent_done:.1f}%")

print("\n✅ All frames and all cameras rendered successfully.")

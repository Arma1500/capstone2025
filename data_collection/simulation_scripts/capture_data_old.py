import bpy
import os
import time

# ---------------------------
# Configuration
# ---------------------------
output_folder = "C:/Work/Capstone/renders"  # Change as needed
start_frame = bpy.context.scene.frame_start
end_frame = bpy.context.scene.frame_end
total_frames = end_frame - start_frame + 1

os.makedirs(output_folder, exist_ok=True)

# ---------------------------
# Set up compositor to save RGB and Depth
# ---------------------------
def setup_compositor_output(base_path):
    scene = bpy.context.scene
    scene.use_nodes = True
    tree = scene.node_tree
    tree.nodes.clear()

    rl = tree.nodes.new('CompositorNodeRLayers')

    rgb_output = tree.nodes.new(type="CompositorNodeOutputFile")
    rgb_output.label = "RGB Output"
    rgb_output.base_path = os.path.join(base_path, "rgb")
    rgb_output.format.file_format = 'PNG'
    rgb_output.file_slots[0].path = "rgb_"

    depth_output = tree.nodes.new(type="CompositorNodeOutputFile")
    depth_output.label = "Depth Output"
    depth_output.base_path = os.path.join(base_path, "depth")
    depth_output.format.file_format = 'PNG'
    depth_output.file_slots[0].path = "depth_"
    depth_output.format.color_mode = 'BW'
    depth_output.format.color_depth = '16'

    tree.links.new(rl.outputs['Image'], rgb_output.inputs[0])
    tree.links.new(rl.outputs['Depth'], depth_output.inputs[0])

# ---------------------------
# Render all frames from all cameras
# ---------------------------
def render_all_cameras():
    cameras = [obj for obj in bpy.data.objects if obj.type == 'CAMERA']
    scene = bpy.context.scene
    total_jobs = len(cameras) * total_frames
    completed_jobs = 0

    print(f"\n🚀 Starting data collection: {len(cameras)} cameras × {total_frames} frames")

    for cam_index, cam in enumerate(cameras):
        print(f"\n🎥 Processing Camera: {cam.name} ({cam_index+1}/{len(cameras)})")
        scene.camera = cam
        base_path = os.path.join(output_folder, cam.name)
        os.makedirs(base_path, exist_ok=True)
        setup_compositor_output(base_path)

        for frame in range(start_frame, end_frame + 1):
            scene.frame_set(frame)
            bpy.ops.render.render(write_still=True)

            # Progress update
            completed_jobs += 1
            percent_done = (completed_jobs / total_jobs) * 100
            print(f"  Frame {frame} — {percent_done:.1f}% complete")

    print("\n✅ Data collection complete. RGB and depth images saved to:")
    print(f"   {output_folder}")

# ---------------------------
# Execute
# ---------------------------
render_all_cameras()
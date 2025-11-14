**data_generation_vis** has a .ipynb file that demonstrates how ground truth and depth image data is processed with sample data of 1 frame from the belnder simulation. While it saves the ground truth pointclouds for 1 frame only to demonstrate how they can be accessed. Everything is properly saved in data generation bulk.

**data_generation_bulk**  runs the same code to process data from all frames - all data is saved in an external drive

Please Note:
If you just want to see the data generatation process, I'd recommend sticking with the vis folder as the main process is the same. Please make sure all file locations are correct before running either. I recommend running from the main capstone2025 folder.

In terms of renders_scaled, these were saved from blender with the scale of 3.8 to 6.8. If you want to save depth renders for your own models and change the scale in blender, please remeber to change it to match in both data_generation_vis and data_generation_bulk.

**blender_scripts** has all scripts used to save the depth renders, original meshes for ray_tracing and the camera parameters. The blender scene and animation are NOT saved here.
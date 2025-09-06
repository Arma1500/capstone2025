# capstone2025
3D Shape Correspondence of Deformable Objects from Sequential Point Clouds

## Current Progress
Data Collection
1. Data saved from blender simulation:
* Depth and RGB images at each frame
* Triangle Mesh at each frame
* Camera Data i.e. intrinsics and extrinsics

2. Ray Tracing to generate the ground truth for each frame:
* Polyscople scene set up using camera data and triangle meshes to match the blender scene
* Ray Tracing using Open3D
* Ground Truth Data for each frame saved as *.json* file

3. Converting Depth images to Point Cloud:
* Obtaining normalised depth values from depth images with OpenCV
* Convert to point cloud with Open3D using camera intrinsics
* Transform using camera extrinsics

## Issues



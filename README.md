# capstone2025
3D Shape Correspondence of Sequential Deforming Objects.  

explaination from capstone report

## Running Requirements
pyFM
Open3D
OpenCV
Polyscope
Meshplot

## Repository Guide??
what the main folders are and what to run

## Progress
Data Collection
1. Data saved from blender simulation:
* Depth and RGB images at each frame
* Triangle Mesh at each frame
* Camera Data i.e. intrinsics and extrinsics

2. Ray Tracing to generate the ground truth for each frame:
* Polyscople scene set up using camera data and triangle meshes to match the blender scene
* Ray Tracing using Open3D
* Ground Truth Data for each frame saved as *.json* file

3. Converting Depth images to Point Cloud to Triangle Meshes:
* Obtaining normalised depth values from depth images with OpenCV
* Convert to point cloud with Open3D using camera intrinsics
* Calculating Normals
* Transform using camera extrinsics
* Converting Render Point Clouds to Triangles meshes with Screened Possion Reconstruction in Meshlab 

Implementation of Consistent ZoomOut with pyFM
4. Obtaining Intial Maps
* Loading meshes and calcuating they eigen basis
* Nearest Neighbour seach with Open3D to calclate inital maps between all meshes in the dataset

5. Creating Functional Map Network and running Consistent ZoomOut with pyFM
* Converting Pointwise maps to Functional maps using pyFM
* Setting Up Functional Map Network with pyFM, where each node is a mesh and the edges are the maps between them.
* Running Consistent ZoomOut refinement with the same parameters as the CZ demo.

Sequential Decay
6. ... explainaing the sequential decay regularisation
7. Editing FMN class to add sequential weights

Evaluation




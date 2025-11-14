# capstone2025
3D Shape Correspondence of Sequential Deforming Objects.  

explaination from capstone report

## Running Requirements
pyFM
Open3D
OpenCV
Polyscope
Meshplot - this isn't in the requirements.txt but it can be found here: https://skoch9.github.io/meshplot/tutorial/#installing-meshplot. If it dosn't work there are alternate plotting fucntions for visualisation in the code.

## Repository Guide

I recommend running everything from here, i.e. capstone2025. Even the file paths in the data_generation folder have been setup in terms of this directory.

### The important files to run are:
1. main.ipynb

        - This is the main file to run.


This is where the FMN and Consistent ZoomOut implementation for the generated meshes is set up. It runs both the original FMN that is installed with pyFM and the FMN_Edited that has been codpied, edited to include the sequential weight decay and saved in this repository. It also includes some visualisation method with both meshplot and open3D. 

2. alternate_main.ipynb 

This is an alternate method where everything is the same except the initial maps which are calculated through intermediate maps based on a specified interval of time, I refer to them in the code as compounded maps. Instead of creating a map directly between frame 1 to frame 4, a map between frame 1 to frame 3 and then frame 3 to frame 4 is used for that.

3. FMN_Edited.py

It is not needed to run this file, it is simply a copy of the original FMN from pyFM where the sequential weight decay has been added. The sections editied are maked with comments in the file. They include
            - set_weights_new() - added function for the sequential weights.
            - zoomout_iteration() and zoomout_refine() - where the set_weights_new() has been added. 

4. data_generation

This includes the data generation processes from obtaining the depth renders from blender to processing and saving them as pointclouds along with their supporting files. More information is avalible inside the folder. Please note that the blender secne is not provided in this repository, just the scripts for saving the depth renders, meshes and camera parameters are. There is also a option just to visualise the data generation process and steps in data_generation_vis.ipynb

### other folders:
1. render_meshes_10 

These are the pre saved triangle meshes converted from the pointclouds saved from data_generation, to be used by main.ipynb and alternate_main.ipynb. Please note, only the file numbers will not correspond to the ones saved in the data_generation folder as those are for example.

2. saved_maps

Where the maps from the FMN models in main.ipynb and alternate_main.ipynb are saved and can be loaded from.

3. archive

All old code, saved just incase.

4. media

Images and files saved during development.


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
7. Editing FMN class to add sequential weights with bonus and interval values.

Evaluation

## References





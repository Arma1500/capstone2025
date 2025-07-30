#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 29 21:28:34 2025

@author: humense
"""
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json


with open('ground_truth/Camera_3/frame_0002.json', 'r') as f:
    camera_hit_list = json.load(f)

# Extract just the hit points
points = [entry['hit'] for entry in camera_hit_list if entry['hit'] is not None]

# Convert to Open3D point cloud
points_np = np.array(points, dtype=np.float32)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_np)

# Visualize
o3d.visualization.draw_geometries([pcd])

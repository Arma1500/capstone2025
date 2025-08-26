# Some of these are not currently updated from when I was working on the blender scene
I think model_save.py and capture_data_2.py are the only ones that are upto date
* capture_data_2.py dosn't have the min and max scale changes yet but besides that it is up to date


note: set_cam_save.py is only the scene with the cameras and saving the intrinsics and extrinsics of the camera data but its an old version
* for saving the extrinsics I'm just using: return np.linalg.inv(cam_obj.matrix_world).tolist()
* then working on changeing the orientations to match the scene the code for ray tracing
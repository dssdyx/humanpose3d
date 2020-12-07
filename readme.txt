step 1: 
Open src/human_pose_3d/src/human_pose_node.cpp 
find "step 1:"
change the fx fy cx cy by sub topic "/camera/aligned_depth_to_color/camera_info" 
the params in K :[fx, 0 , cx, 0, fy, cy, 0, 0, 1]
then catkin_make to complie

step 2: 
roscd realsense2_camera/launch
sudo gedit rs_camera.launch
Change the "align_depth" to true

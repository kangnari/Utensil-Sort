# Utensil-Sort
Final project for EE106A (Introduction to Robotics).

Setup steps
1. roscore

(Instructions in lab 6)
2. roslaunch odroid_machine remote_zumy.launch mname:=zumyXX

(Instructions in lab 4; look at lab4_cam/run_cam.launch)
3. roslaunch kalman_zumy run_all.launch

(To look at camera output)
4. rosrun image_view image_view image:=/usb_cam_[global/utensil]/image_raw

(To open tf viewer)
5.rosrun rviz rviz

6. Set up two camera services (camera_srv.py). Arguments: (utensil, global). The path planning node 
will use the image from camera_srv/usb_cam_global to do path planning to destination. 


TODO
1. Change any references to usb_cam/... to usb_cam_utensil/... or usb_cam_global/...
2. Create a script to open all necessary nodes (camera, etc., likely in run_all.launch)
3. Translate corner detection to numpy
4. Get pictures of forks, knives, and spoons
5. Eat somewhere so we can steal forks, knives, and spoons
6. Handle all XXX

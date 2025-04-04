 . install/setup.bash
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.015 --ros-args -r image:=/camera/image_raw 

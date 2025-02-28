ros2 run face_tracker_pkg face_tracker_controller --ros-args \
  -p servomaxx:=0.5 \
  -p servomaxy:=0.5 \
  -p servomin:=-0.5 \
  -p screenmaxx:=640 \
  -p screenmaxy:=480 \
  -p center_offset:=100 \
  -p center_offsety:=60 \
  -p step_distancex:=0.005 \
  -p step_distancey:=0.005

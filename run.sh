docker run -it --rm \
  --net=host \
  --privileged \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  inria_docker:april_tag_humble \
  bash -c "source /ros2_ws/install/setup.bash && ros2 launch apriltag_pose aprlitag_tiago_pro.py"
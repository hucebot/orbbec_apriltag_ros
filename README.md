# Orbbec AprilTag ROS

This project utilizes the Orbbec Femto Bold RGB-D camera to detect 6D pose fiducial markers and publish them to ROS1:
* Orbbec Femto Bolt Camera: https://www.orbbec.com/products/tof-camera/femto-bolt/
* Orbbec SDK: https://github.com/orbbec/OrbbecSDK
* AprilTags Library: https://april.eecs.umich.edu/software/apriltag

AprilTags detects the tag corners in the color image, and the point cloud is used to estimate the 3D position and orientation of the tag in the camera frame. Each detected tag's pose is published as a `geometry_msgs/PoseStamped` message in ROS1.

## Docker

Build the Docker image using:
```
 ./build.sh
```
Open a terminal in the Docker container for debugging or development:
```
./launch_bash.sh
```
Start the publisher directly in the Docker container using:
```
./launch_publisher.sh [is_display] [is_verbose] [ros_master_ip] [this_node_ip]
```

## Usage

```
./inria_orbbec_tags [is_display, default=0] [is_verbose, default=0] [ros_master_ip, default=127.0.0.1] [this_node_ip, default=127.0.0.1]
```
`is_display`: Enables the visualization of the color and depth camera, as well as the detected tags (0 or 1).
`is_verbose`: Enables printing additional information to the standard output (0 or 1).
`ros_master_ip`: Defines the IP address for the ROS Master (string).
`this_node_ip`: Assigns the IP address for this ROS node (string).
Note that the ROS Master must be running externally.

## ROS Topic

For each detected AprilTag marker, the publisher outputs a `geometry_msgs/PoseStamped` message as fast as possible, up to 30Hz, with the topic name `/inria_orbbec_tags/pose_tag_{TagID}`.

## License

Licensed under the [BSD License](LICENSE)


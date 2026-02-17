# AprilTag Pose

Detect AprilTag fiducial markers from any RGB-D source and publish their 6D poses via ROS 2. The node subscribes to a color image and an organized colored point cloud, making it compatible with any camera that publishes these standard ROS 2 topics (Orbbec, RealSense, ZED, Kinect, etc.).

* AprilTags Library: https://april.eecs.umich.edu/software/apriltag
* ROS 2 Humble on Ubuntu 22.04

AprilTags detects the tag corners in the color image, and the organized point cloud is used to estimate the 3D position and orientation of the tag. Each detected tag's pose is published as a `geometry_msgs/msg/PoseStamped` message.

## Docker

Build the Docker image:
```
./build.sh
```
Run the container:
```
./run.sh
```
Inside the container, launch the node:
```
ros2 launch apriltag_pose apriltag_pose.launch.py
```

## Usage

With verbose and display
```
ros2 launch apriltag_pose apriltag_pose.launch.py display:=true verbose:=true
```

With custom topics:
```
ros2 launch apriltag_pose apriltag_pose.launch.py image_topic:=/my_camera/color/image_raw cloud_topic:=/my_camera/depth/color/points
```

With another output `frame_id`
```
ros2 launch apriltag_pose apriltag_pose.launch.py image_topic:=/tiago_head_camera_down/color/image_raw cloud_topic:=/tiago_head_camera_down/depth_registered/points display:=true verbose:=true frame_id:=tiago_head_camera_down_color_optical_frame
```

## ROS 2 Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `verbose` | bool | `false` | Enable verbose timing output |
| `display` | bool | `false` | Enable OpenCV visualization of detected tags |
| `apply_coordinate_transform` | bool | `false` | Apply (x,y,z) to (z,-x,-y) coordinate transform |
| `frame_id` | string | `camera_color_optical_frame` | TF frame_id for published poses |
| `image_topic` | string | `/camera/camera/color/image_raw` | Color image topic to subscribe to |
| `cloud_topic` | string | `/camera/camera/depth/color/points` | Organized point cloud topic to subscribe to |
| `sync_queue_size` | int | `30` | Queue size for approximate time synchronization |

## ROS 2 Topics

### Subscribed
- `image_topic` (`sensor_msgs/msg/Image`): Color image
- `cloud_topic` (`sensor_msgs/msg/PointCloud2`): Organized colored point cloud, aligned to the color image

### Published
For each detected AprilTag marker, a `geometry_msgs/msg/PoseStamped` message is published on `apriltag_pose/pose_tag_{TagID}`.

## Requirements

The point cloud must be:
- **Organized**: `height > 1`, with dimensions matching the color image
- **Aligned**: Depth-to-color registered so that pixel (x, y) in the image corresponds to the point at (x, y) in the point cloud

Most RGB-D camera ROS 2 drivers provide an aligned/registered point cloud topic (e.g. `camera/depth_registered/points`).

## Building in an Existing Workspace

To add this package to an existing ROS 2 workspace:
```bash
cd /ros2_ws/src
cp -r /path/to/apriltag_pose .
cd /ros2_ws
colcon build --packages-select apriltag_pose
source install/setup.bash
```

Note: The AprilTag library must be built at `/opt/apriltag` (see Dockerfile for reference).

## License

Licensed under the [BSD License](LICENSE)

# AprilTag Pose

Detect AprilTag fiducial markers from any RGB-D source and publish their 6D poses via ROS 2. The node subscribes to an organized colored point cloud, making it compatible with any camera that publishes this standard ROS 2 topic (Orbbec, RealSense, ZED, Kinect, etc.).

* AprilTags Library: https://april.eecs.umich.edu/software/apriltag
* ROS 2 Humble on Ubuntu 22.04

AprilTags detects the tag corners in the color image extracted from the point cloud, and the organized point cloud is used to estimate the 3D position and orientation of the tag. Each detected tag's pose is published as a `geometry_msgs/msg/PoseStamped` message.

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

```
ros2 launch apriltag_pose apriltag_pose.launch.py display:=true verbose:=true
```
With a custom topic:
```
ros2 launch apriltag_pose apriltag_pose.launch.py cloud_topic:=/my_camera/depth/color/points
```
Publishing poses in a different frame:
```
ros2 launch apriltag_pose apriltag_pose.launch.py publishing_frame:=base_link publish_tf:=true
```

## ROS 2 Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `verbose` | bool | `false` | Enable verbose timing output |
| `display` | bool | `false` | Enable OpenCV visualization of detected tags |
| `cloud_topic` | string | `/camera/camera/depth/color/points` | Organized colored point cloud topic to subscribe to |
| `publish_tf` | bool | `false` | Broadcast a TF frame for each detected tag |
| `tag_frame_prefix` | string | `apriltag` | Prefix for per-tag TF child frames (e.g. `apriltag_0`, `apriltag_1`) |
| `publishing_frame` | string | `""` | If set, transform poses from camera frame to this frame before publishing. If empty, publish in the point cloud's frame |
| `transform_timeout` | double | `0.1` | Timeout in seconds for TF lookup when `publishing_frame` is set |
| `filter_type` | string | `none` | Pose filter type: `none` or `median` |
| `filter_window` | int | `5` | Filter window size (number of frames) |

## ROS 2 Topics

### Subscribed
- `cloud_topic` (`sensor_msgs/msg/PointCloud2`): Organized colored point cloud

### Published
- `apriltag_pose/pose_tag_{id}` (`geometry_msgs/msg/PoseStamped`): Pose for each detected tag
- `apriltag_pose/filtered_pose_tag_{id}` (`geometry_msgs/msg/PoseStamped`): Filtered pose (when `filter_type` is set)

### TF Broadcasts (when `publish_tf:=true`)
- `<publishing_frame or cloud_frame>` -> `<tag_frame_prefix>_{id}`

## How It Works

1. The node extracts a grayscale image from the point cloud's RGB data
2. AprilTag detection runs on the grayscale image to find tag corners
3. The 3D positions of the tag corners are looked up in the point cloud
4. The tag's 6D pose (position + orientation) is computed in the camera frame
5. If `publishing_frame` is set, the pose is transformed to that frame via TF
6. The pose is published; if `publish_tf` is enabled, the TF is broadcast

## Requirements

The point cloud must be:
- **Organized**: `height > 1`, with dimensions matching the color image
- **Colored**: Must contain an `rgb` field
- **Aligned**: Depth-to-color registered so that pixel (x, y) in the image corresponds to the point at (x, y) in the point cloud

Most RGB-D camera ROS 2 drivers provide an aligned/registered colored point cloud topic (e.g. `camera/depth/color/points`).

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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('display', default_value='false'),
        DeclareLaunchArgument('apply_transform', default_value='false'),
        DeclareLaunchArgument('frame_id', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('cloud_topic', default_value='/camera/camera/depth/color/points'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('tag_frame_prefix', default_value='apriltag',
            description='Prefix for per-tag TF child frames (e.g. apriltag_0, apriltag_1)'),
        DeclareLaunchArgument('tf_parent_frame', default_value='',
            description='If set, publish TF from this frame to each tag frame (chained through camera frame). If empty, publish from camera frame directly.'),
        DeclareLaunchArgument('transform_timeout', default_value='0.1',
            description='Timeout in seconds for TF lookup when tf_parent_frame is set'),
        DeclareLaunchArgument('filter_type', default_value='none',
            description='Pose filter type: "none" or "median"'),
        DeclareLaunchArgument('filter_window', default_value='5',
            description='Filter window size (number of frames)'),

        Node(
            package='apriltag_pose',
            executable='apriltag_pose',
            name='apriltag_pose',
            output='screen',
            parameters=[{
                'verbose': LaunchConfiguration('verbose'),
                'display': LaunchConfiguration('display'),
                'apply_coordinate_transform': LaunchConfiguration('apply_transform'),
                'frame_id': LaunchConfiguration('frame_id'),
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'tag_frame_prefix': LaunchConfiguration('tag_frame_prefix'),
                'tf_parent_frame': LaunchConfiguration('tf_parent_frame'),
                'transform_timeout': LaunchConfiguration('transform_timeout'),
                'filter_type': LaunchConfiguration('filter_type'),
                'filter_window': LaunchConfiguration('filter_window'),
            }],
        ),
    ])

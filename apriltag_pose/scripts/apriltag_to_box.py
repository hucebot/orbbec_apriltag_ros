#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R

import tf2_ros


class ApriltagToBoxNode(Node):
    def __init__(self):
        super().__init__("apriltag_to_box")

        self.declare_parameter("apriltag_pose_topic", "apriltag_pose/pose_tag_0")
        self.declare_parameter("box_topic", "/g1pilot/box_pose")
        self.declare_parameter("publishing_frame", "")
        self.declare_parameter("transform_timeout", 0.1)
        # Asymmetric extents: x_minus,x_plus,y_minus,y_plus,z_minus,z_plus (meters)
        self.declare_parameter("box_extents", "0.01,0.0,0.5,0.50,0.5,0.5")

        apriltag_topic = self.get_parameter("apriltag_pose_topic").value
        box_topic = self.get_parameter("box_topic").value
        self.publishing_frame = self.get_parameter("publishing_frame").value
        self.transform_timeout = self.get_parameter("transform_timeout").value

        extents = [float(v) for v in self.get_parameter("box_extents").value.split(",")]
        if len(extents) != 6:
            self.get_logger().error(f"box_extents must have 6 values, got {len(extents)}")
            extents = [0.02, 0.0, 1.09, 0.30, 0.25, 0.56]
        self.x_minus, self.x_plus = extents[0], extents[1]
        self.y_minus, self.y_plus = extents[2], extents[3]
        self.z_minus, self.z_plus = extents[4], extents[5]

        # TF listener for frame transformation
        if self.publishing_frame:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.get_logger().info(
                f"Box marker will be transformed to frame: '{self.publishing_frame}'"
            )

        self.subscription = self.create_subscription(
            PoseStamped, apriltag_topic, self.apriltag_callback, 10
        )
        self.publisher = self.create_publisher(Marker, box_topic, 10)

        self.get_logger().info(
            f"Subscribed to '{apriltag_topic}', publishing box on '{box_topic}'"
        )

    def apriltag_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w])

        # Box dimensions from asymmetric extents
        size_x = self.x_minus + self.x_plus
        size_y = self.y_minus + self.y_plus
        size_z = self.z_minus + self.z_plus

        # Box center offset in the apriltag's local frame
        offset_local = np.array([
            (-self.x_minus + self.x_plus) / 2.0,
            (-self.y_minus + self.y_plus) / 2.0,
            (-self.z_minus + self.z_plus) / 2.0,
        ])

        # Transform offset to parent frame and add to tag position
        offset_parent = rot.apply(offset_local)
        center = np.array([msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z]) + offset_parent

        frame_id = msg.header.frame_id

        # Transform to publishing_frame if set
        if self.publishing_frame and frame_id != self.publishing_frame:
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.publishing_frame,
                    frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=self.transform_timeout),
                )
                tf_t = tf_stamped.transform.translation
                tf_q = tf_stamped.transform.rotation
                tf_rot = R.from_quat([tf_q.x, tf_q.y, tf_q.z, tf_q.w])
                tf_trans = np.array([tf_t.x, tf_t.y, tf_t.z])

                center = tf_rot.apply(center) + tf_trans
                rot = tf_rot * rot
                q_arr = rot.as_quat()  # [x, y, z, w]
                frame_id = self.publishing_frame
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(
                    f"Could not transform {frame_id} -> {self.publishing_frame}: {ex}. "
                    f"Publishing in {frame_id}.",
                    throttle_duration_sec=5.0,
                )
                q_arr = None
        else:
            q_arr = None

        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = frame_id
        marker.ns = "box_obstacle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = size_x
        marker.scale.y = size_y
        marker.scale.z = size_z
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        if q_arr is not None:
            marker.pose.orientation.x = q_arr[0]
            marker.pose.orientation.y = q_arr[1]
            marker.pose.orientation.z = q_arr[2]
            marker.pose.orientation.w = q_arr[3]
        else:
            marker.pose.orientation = q
        marker.color.r = 0.8
        marker.color.g = 0.5
        marker.color.b = 0.2
        marker.color.a = 0.5

        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagToBoxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

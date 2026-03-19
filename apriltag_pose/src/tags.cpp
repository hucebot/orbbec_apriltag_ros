#include <iostream>
#include <map>
#include <cmath>
#include <cstring>
#include <chrono>
#include <functional>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.hpp>

extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
}

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode()
        : Node("apriltag_pose"), frameIndex_(0)
    {
        //Declare and load parameters
        this->declare_parameter("verbose", false);
        this->declare_parameter("display", false);
        this->declare_parameter("input_mode", "pointcloud");  // "pointcloud" or "rgbd"
        this->declare_parameter("cloud_topic", "/camera/camera/depth/color/points");
        this->declare_parameter("image_topic", "/camera/color/image_raw");
        this->declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw");
        this->declare_parameter("camera_info_topic", "/camera/aligned_depth_to_color/camera_info");
        this->declare_parameter("depth_scale", 0.001);  // depth units to meters
        this->declare_parameter("publish_tf", false);
        this->declare_parameter("tag_frame_prefix", "apriltag");
        this->declare_parameter("publishing_frame", "");
        this->declare_parameter("transform_timeout", 0.1);
        this->declare_parameter("min_decision_margin", 0.0);
        this->declare_parameter("fix_normal_axis", "");  // fix tag normal to parent frame axis: "x","-x","y","-y","z","-z" or "" (disabled)
        this->declare_parameter("debug", false);
        is_verbose_ = this->get_parameter("verbose").as_bool();
        is_display_ = this->get_parameter("display").as_bool();
        input_mode_ = this->get_parameter("input_mode").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        tag_frame_prefix_ = this->get_parameter("tag_frame_prefix").as_string();
        publishing_frame_ = this->get_parameter("publishing_frame").as_string();
        transform_timeout_ = this->get_parameter("transform_timeout").as_double();
        depth_scale_ = this->get_parameter("depth_scale").as_double();
        min_decision_margin_ = this->get_parameter("min_decision_margin").as_double();
        fix_normal_axis_ = this->get_parameter("fix_normal_axis").as_string();
        is_debug_ = this->get_parameter("debug").as_bool();

        // Parse and validate fix_normal_axis
        if (!fix_normal_axis_.empty()) {
            if (fix_normal_axis_ == "x")       fixedNormal_ = Eigen::Vector3d( 1, 0, 0);
            else if (fix_normal_axis_ == "-x")  fixedNormal_ = Eigen::Vector3d(-1, 0, 0);
            else if (fix_normal_axis_ == "y")   fixedNormal_ = Eigen::Vector3d( 0, 1, 0);
            else if (fix_normal_axis_ == "-y")  fixedNormal_ = Eigen::Vector3d( 0,-1, 0);
            else if (fix_normal_axis_ == "z")   fixedNormal_ = Eigen::Vector3d( 0, 0, 1);
            else if (fix_normal_axis_ == "-z")  fixedNormal_ = Eigen::Vector3d( 0, 0,-1);
            else {
                RCLCPP_ERROR(this->get_logger(),
                    "Invalid fix_normal_axis '%s'. Must be x,-x,y,-y,z,-z or empty. Disabling.",
                    fix_normal_axis_.c_str());
                fix_normal_axis_ = "";
            }
            if (!fix_normal_axis_.empty()) {
                RCLCPP_INFO(this->get_logger(),
                    "Tag normal will be fixed to '%s' axis of publishing frame",
                    fix_normal_axis_.c_str());
            }
        }

        //Initialize AprilTag detector
        tagFamily_ = tag36h11_create();
        tagDetector_ = apriltag_detector_create();
        apriltag_detector_add_family(tagDetector_, tagFamily_);
        tagDetector_->quad_decimate = 1.0;
        tagDetector_->quad_sigma = 0.0;
        tagDetector_->refine_edges = 1;
        tagDetector_->decode_sharpening = 0.25;
        tagDetector_->nthreads = 4;
        tagDetector_->debug = 0;

        //Setup display window
        if (is_display_) {
            cv::namedWindow("color", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        }

        //Setup TF listener for parent frame transform (needed for pose publishing)
        if (!publishing_frame_.empty()) {
            tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
            RCLCPP_INFO(this->get_logger(),
                "Poses will be transformed to parent frame: %s", publishing_frame_.c_str());
        }

        //Setup optional TF broadcaster
        if (publish_tf_) {
            tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            if (!publishing_frame_.empty()) {
                RCLCPP_INFO(this->get_logger(),
                    "TF broadcasting enabled: %s -> %s_<id>",
                    publishing_frame_.c_str(), tag_frame_prefix_.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "TF broadcasting enabled: <cloud_frame> -> %s_<id>",
                    tag_frame_prefix_.c_str());
            }
        }

        //Setup debug publishers
        if (is_debug_) {
            debugCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "apriltag_pose/debug/points", 10);
            debugSegPub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "apriltag_pose/debug/segmentation", 10);
            debugGrayPub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "apriltag_pose/debug/grayscale", 10);
            debugDetectionPub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "apriltag_pose/debug/detections", 10);
            debugDepthCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "apriltag_pose/debug/depth_cloud", 10);
            RCLCPP_INFO(this->get_logger(), "Debug publishers enabled");
        }

        //Setup subscriptions based on input mode
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        qos_profile.depth = 10;

        if (input_mode_ == "rgbd") {
            std::string image_topic = this->get_parameter("image_topic").as_string();
            std::string depth_topic = this->get_parameter("depth_topic").as_string();
            std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();

            // Camera info subscription (async, cached)
            subCameraInfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile)),
                std::bind(&AprilTagNode::cameraInfoCallback, this, std::placeholders::_1));

            // Synchronized RGB + Depth
            subImage_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, image_topic, rmw_qos_profile_default);
            subDepth_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, depth_topic, rmw_qos_profile_default);
            sync_ = std::make_shared<Sync>(SyncPolicy(10), *subImage_, *subDepth_);
            sync_->registerCallback(
                std::bind(&AprilTagNode::rgbdCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "AprilTag detector initialized in RGBD mode. Subscribing to:");
            RCLCPP_INFO(this->get_logger(), "  Image: %s", image_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "  Depth: %s", depth_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "  CameraInfo: %s", camera_info_topic.c_str());
        } else {
            std::string cloud_topic = this->get_parameter("cloud_topic").as_string();
            subCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                cloud_topic,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile)),
                std::bind(&AprilTagNode::pointcloudCallback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "AprilTag detector initialized in PointCloud mode. Subscribing to:");
            RCLCPP_INFO(this->get_logger(), "  PointCloud: %s", cloud_topic.c_str());
        }

        timeProcessLoop_ = std::chrono::high_resolution_clock::now();
    }

    ~AprilTagNode()
    {
        apriltag_detector_destroy(tagDetector_);
        tag36h11_destroy(tagFamily_);
        if (is_display_) {
            cv::destroyAllWindows();
        }
    }

    bool isDisplay() const { return is_display_; }

private:

    //Camera-frame pose of a detected tag (before any coordinate transform)
    struct CamTagPose {
        int id;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

    // ---- Shared detection and pose pipeline ----

    // ---- RANSAC plane fitting ----

    struct PlaneResult {
        bool success;
        Eigen::Vector3d normal;
        double d;
        int inliers;
        double inlierRatio;
    };

    PlaneResult fitPlaneRANSAC(
        const std::vector<Eigen::Vector3d>& points,
        int iterations = 100,
        double threshold = 0.005)
    {
        PlaneResult result{false, Eigen::Vector3d(0, 0, 1), 0.0, 0, 0.0};
        std::mt19937 rng(42);

        for (int iter = 0; iter < iterations; iter++) {
            std::uniform_int_distribution<int> dist(0, points.size() - 1);
            int i1 = dist(rng), i2 = dist(rng), i3 = dist(rng);
            if (i1 == i2 || i1 == i3 || i2 == i3) continue;

            Eigen::Vector3d n = (points[i2] - points[i1]).cross(points[i3] - points[i1]);
            if (n.norm() < 1e-10) continue;
            n.normalize();
            double d = -n.dot(points[i1]);

            int inliers = 0;
            for (const auto& pt : points) {
                if (std::abs(n.dot(pt) + d) < threshold) inliers++;
            }

            if (inliers > result.inliers) {
                result.inliers = inliers;
                result.normal = n;
                result.d = d;
            }
        }

        result.inlierRatio = static_cast<double>(result.inliers) / points.size();
        result.success = result.inlierRatio >= 0.5;
        if (!result.success) return result;

        // Refine plane normal with SVD on inliers
        // Collect inliers and compute centroid
        std::vector<Eigen::Vector3d> inlierPts;
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& pt : points) {
            if (std::abs(result.normal.dot(pt) + result.d) < threshold) {
                inlierPts.push_back(pt);
                centroid += pt;
            }
        }
        centroid /= inlierPts.size();

        // Build matrix of centered inlier points (N x 3)
        Eigen::MatrixXd A(inlierPts.size(), 3);
        for (size_t i = 0; i < inlierPts.size(); i++) {
            A.row(i) = (inlierPts[i] - centroid).transpose();
        }

        // SVD: smallest singular value's right singular vector = plane normal
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
        result.normal = svd.matrixV().col(2).normalized();
        result.d = -result.normal.dot(centroid);

        //Ensure normal points toward camera (negative z direction)
        if (result.normal.z() > 0) {
            result.normal = -result.normal;
            result.d = -result.d;
        }
        return result;
    }

    // ---- Median position from plane inliers ----

    Eigen::Vector3d computeMedianInlierPosition(
        const std::vector<Eigen::Vector3d>& points,
        const Eigen::Vector3d& normal,
        double d,
        double threshold)
    {
        std::vector<double> xs, ys, zs;
        for (const auto& pt : points) {
            if (std::abs(normal.dot(pt) + d) < threshold) {
                xs.push_back(pt.x());
                ys.push_back(pt.y());
                zs.push_back(pt.z());
            }
        }
        std::sort(xs.begin(), xs.end());
        std::sort(ys.begin(), ys.end());
        std::sort(zs.begin(), zs.end());
        return Eigen::Vector3d(
            xs[xs.size() / 2],
            ys[ys.size() / 2],
            zs[zs.size() / 2]);
    }

    // ---- Ray-plane intersection ----

    Eigen::Vector3d rayPlaneIntersect(
        double px, double py,
        const Eigen::Vector3d& planeNormal, double planeD,
        const Eigen::Vector3d& fallback,
        std::function<Eigen::Vector3d(int, int)> getPoint,
        std::function<bool(const Eigen::Vector3d&)> isValid)
    {
        Eigen::Vector3d rayDir;
        if (cameraInfo_) {
            double fx = cameraInfo_->k[0];
            double fy = cameraInfo_->k[4];
            double cx = cameraInfo_->k[2];
            double cy = cameraInfo_->k[5];
            rayDir = Eigen::Vector3d((px - cx) / fx, (py - cy) / fy, 1.0).normalized();
        } else {
            Eigen::Vector3d pt = getPoint((int)px, (int)py);
            if (isValid(pt)) {
                rayDir = pt.normalized();
            } else {
                rayDir = fallback.normalized();
            }
        }
        double denom = planeNormal.dot(rayDir);
        if (std::abs(denom) < 1e-10) return fallback;
        double t = -planeD / denom;
        return rayDir * t;
    }

    // ---- Compute tag orientation from plane normal and edge ----

    Eigen::Quaterniond computeTagOrientation(
        const apriltag_detection_t* det,
        const Eigen::Vector3d& planeNormal, double planeD,
        const Eigen::Vector3d& tagCenter,
        std::function<Eigen::Vector3d(int, int)> getPoint,
        std::function<bool(const Eigen::Vector3d&)> isValid)
    {
        // Project all 4 corners onto RANSAC plane
        Eigen::Vector3d p0 = rayPlaneIntersect(
            det->p[0][0], det->p[0][1], planeNormal, planeD, tagCenter, getPoint, isValid);
        Eigen::Vector3d p1 = rayPlaneIntersect(
            det->p[1][0], det->p[1][1], planeNormal, planeD, tagCenter, getPoint, isValid);
        Eigen::Vector3d p2 = rayPlaneIntersect(
            det->p[2][0], det->p[2][1], planeNormal, planeD, tagCenter, getPoint, isValid);
        Eigen::Vector3d p3 = rayPlaneIntersect(
            det->p[3][0], det->p[3][1], planeNormal, planeD, tagCenter, getPoint, isValid);

        // Average both parallel edges for Z and Y directions
        Eigen::Vector3d rawZ = ((p0 - p1) + (p3 - p2)).normalized();
        Eigen::Vector3d rawY = -((p2 - p1) + (p3 - p0)).normalized();

        // Project Z onto the SVD-refined plane, then derive Y via cross product
        Eigen::Vector3d vectZ_cam = (rawZ - rawZ.dot(planeNormal) * planeNormal).normalized();
        Eigen::Vector3d vectY_cam = planeNormal.cross(vectZ_cam).normalized();
        // Ensure Y sense matches the corner-derived Y
        if (vectY_cam.dot(rawY) < 0) vectY_cam = -vectY_cam;
        // X from cross product (guaranteed orthogonal triad)
        Eigen::Vector3d vectX_cam = vectY_cam.cross(vectZ_cam).normalized();

        Eigen::Matrix3d rotCam;
        rotCam.col(0) = -vectX_cam;
        rotCam.col(1) = -vectZ_cam;
        rotCam.col(2) = -vectY_cam;
        Eigen::Quaterniond q(rotCam);
        q.normalize();
        return q;
    }

    // ---- Fix tag normal to a parent-frame axis ----
    // The tag's X axis (col 0 of rotation matrix) is the surface normal.
    // This replaces it with the fixed axis and reorthogonalizes.

    Eigen::Quaterniond fixNormalAxis(const Eigen::Quaterniond& orientation) const
    {
        Eigen::Matrix3d R = orientation.toRotationMatrix();

        // col(0) is the tag normal, col(1) and col(2) are in-plane directions
        // Replace normal with the fixed axis
        Eigen::Vector3d newNormal = fixedNormal_;

        // Keep col(1) as close to original as possible: project onto plane perpendicular to new normal
        Eigen::Vector3d rawY = R.col(1);
        Eigen::Vector3d newY = (rawY - rawY.dot(newNormal) * newNormal).normalized();

        // If rawY was nearly parallel to newNormal, fall back to col(2)
        if (newY.hasNaN() || newY.norm() < 0.5) {
            Eigen::Vector3d rawZ = R.col(2);
            Eigen::Vector3d newZ = (rawZ - rawZ.dot(newNormal) * newNormal).normalized();
            newY = newZ.cross(newNormal).normalized();
        }

        // Complete the orthogonal triad
        Eigen::Vector3d newZ = newNormal.cross(newY).normalized();

        Eigen::Matrix3d Rfixed;
        Rfixed.col(0) = newNormal;
        Rfixed.col(1) = newY;
        Rfixed.col(2) = newZ;

        Eigen::Quaterniond q(Rfixed);
        q.normalize();
        return q;
    }

    // ---- Collect valid 3D points inside tag quad ----

    std::vector<Eigen::Vector3d> collectTagPoints(
        const apriltag_detection_t* det,
        int width, int height,
        std::function<Eigen::Vector3d(int, int)> getPoint,
        std::function<bool(const Eigen::Vector3d&)> isValid)
    {
        std::vector<cv::Point2f> quad = {
            cv::Point2f(det->p[0][0], det->p[0][1]),
            cv::Point2f(det->p[1][0], det->p[1][1]),
            cv::Point2f(det->p[2][0], det->p[2][1]),
            cv::Point2f(det->p[3][0], det->p[3][1]),
        };
        cv::Rect bbox = cv::boundingRect(quad);
        int xmin = std::max(0, bbox.x);
        int ymin = std::max(0, bbox.y);
        int xmax = std::min(width - 1, bbox.x + bbox.width);
        int ymax = std::min(height - 1, bbox.y + bbox.height);

        std::vector<Eigen::Vector3d> points;
        for (int py = ymin; py <= ymax; py++) {
            for (int px = xmin; px <= xmax; px++) {
                if (cv::pointPolygonTest(quad, cv::Point2f(px, py), false) >= 0) {
                    Eigen::Vector3d pt = getPoint(px, py);
                    if (isValid(pt)) {
                        points.push_back(pt);
                    }
                }
            }
        }
        return points;
    }

    // ---- Look up publishing frame transform ----

    struct FrameTransform {
        bool valid;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d translation;
        std::string frame_id;
    };

    FrameTransform lookupPublishingFrame(const std::string& camera_frame)
    {
        FrameTransform tf{false, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), camera_frame};
        if (publishing_frame_.empty() || !tfBuffer_) return tf;

        try {
            auto tfStamped = tfBuffer_->lookupTransform(
                publishing_frame_, camera_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(transform_timeout_));
            tf.rotation = Eigen::Quaterniond(
                tfStamped.transform.rotation.w,
                tfStamped.transform.rotation.x,
                tfStamped.transform.rotation.y,
                tfStamped.transform.rotation.z);
            tf.translation = Eigen::Vector3d(
                tfStamped.transform.translation.x,
                tfStamped.transform.translation.y,
                tfStamped.transform.translation.z);
            tf.frame_id = publishing_frame_;
            tf.valid = true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Could not look up transform %s -> %s: %s. Publishing in camera frame.",
                publishing_frame_.c_str(), camera_frame.c_str(), ex.what());
        }
        return tf;
    }

    // ---- Publish tag pose ----

    void publishTagPose(
        const std_msgs::msg::Header& header,
        const std::string& frame_id,
        int tagId,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        float confidence,
        float inlierRatio,
        int numPoints)
    {
        std::string tagStr = "tag_" + std::to_string(tagId);
        if (containerPub_.count(tagId) == 0) {
            containerPub_[tagId] = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "apriltag_pose/pose_" + tagStr, 10);
            metricsPub_[tagId]["decision_margin"] = this->create_publisher<std_msgs::msg::Float32>(
                "apriltag_pose/metrics/decision_margin/" + tagStr, 10);
            metricsPub_[tagId]["inlier_ratio"] = this->create_publisher<std_msgs::msg::Float32>(
                "apriltag_pose/metrics/inlier_ratio/" + tagStr, 10);
            metricsPub_[tagId]["num_points"] = this->create_publisher<std_msgs::msg::Float32>(
                "apriltag_pose/metrics/num_points/" + tagStr, 10);
        }

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = header.stamp;
        msg.header.frame_id = frame_id;
        msg.pose.position.x = position.x();
        msg.pose.position.y = position.y();
        msg.pose.position.z = position.z();
        msg.pose.orientation.x = orientation.x();
        msg.pose.orientation.y = orientation.y();
        msg.pose.orientation.z = orientation.z();
        msg.pose.orientation.w = orientation.w();
        containerPub_[tagId]->publish(msg);

        auto publishMetric = [&](const std::string& name, float value) {
            std_msgs::msg::Float32 m;
            m.data = value;
            metricsPub_[tagId][name]->publish(m);
        };
        publishMetric("decision_margin", confidence);
        publishMetric("inlier_ratio", inlierRatio);
        publishMetric("num_points", static_cast<float>(numPoints));
    }

    // ---- Draw detections on image ----

    void drawDetections(
        cv::Mat& matColorBGR,
        zarray_t* detections,
        const std::vector<bool>& is_pose_detected)
    {
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            //Draw plane points if tag pose is detected
            if (is_pose_detected.at(i)) {
                Eigen::Matrix3d homography;
                homography(0, 0) = det->H->data[0];
                homography(0, 1) = det->H->data[1];
                homography(0, 2) = det->H->data[2];
                homography(1, 0) = det->H->data[3];
                homography(1, 1) = det->H->data[4];
                homography(1, 2) = det->H->data[5];
                homography(2, 0) = det->H->data[6];
                homography(2, 1) = det->H->data[7];
                homography(2, 2) = det->H->data[8];
                for (double x = -1.0; x <= 1.0; x += 1.0 / 4.0) {
                    for (double y = -1.0; y <= 1.0; y += 1.0 / 4.0) {
                        Eigen::Vector3d uv1(x, y, 1.0);
                        Eigen::Vector3d uv2 = homography * uv1;
                        uv2 = uv2 * (1.0 / uv2.z());
                        cv::circle(matColorBGR,
                            cv::Point((int)uv2.x(), (int)uv2.y()),
                            3, cv::Scalar(255, 0, 255), -1);
                    }
                }
            }

            cv::circle(matColorBGR, cv::Point(det->p[0][0], det->p[0][1]),
                5, cv::Scalar(0, 0, 255), -1);
            cv::circle(matColorBGR, cv::Point(det->p[1][0], det->p[1][1]),
                5, cv::Scalar(0, 255, 0), -1);
            cv::circle(matColorBGR, cv::Point(det->p[2][0], det->p[2][1]),
                5, cv::Scalar(255, 0, 0), -1);
            cv::circle(matColorBGR, cv::Point(det->p[3][0], det->p[3][1]),
                5, cv::Scalar(255, 255, 0), -1);
            cv::circle(matColorBGR, cv::Point(det->c[0], det->c[1]),
                5, cv::Scalar(0, 255, 255), -1);
        }
    }

    // ---- Publish debug visualizations ----

    void publishDebugViz(
        zarray_t* detections,
        const std::vector<bool>& is_pose_detected,
        const std_msgs::msg::Header& header,
        const std::string& camera_frame,
        const cv::Mat& matColorGray,
        cv::Mat& matColorBGR,
        int width, int height,
        std::function<Eigen::Vector3d(int, int)> getPoint,
        std::function<bool(const Eigen::Vector3d&)> isValid)
    {
        //Debug point cloud and segmentation
        if (is_debug_ && zarray_size(detections) > 0) {
            static const uint8_t tagColors[][3] = {
                {255, 0, 0}, {0, 255, 0}, {0, 0, 255},
                {255, 255, 0}, {0, 255, 255}, {255, 0, 255},
            };
            static const int nColors = 6;

            cv::Mat segMask(height, width, CV_8UC1, cv::Scalar(0));
            std::vector<float> cloudPoints;

            for (int i = 0; i < zarray_size(detections); i++) {
                if (!is_pose_detected.at(i)) continue;

                apriltag_detection_t* det;
                zarray_get(detections, i, &det);

                std::vector<cv::Point> quad(4);
                for (int j = 0; j < 4; j++)
                    quad[j] = cv::Point((int)det->p[j][0], (int)det->p[j][1]);

                cv::fillConvexPoly(segMask, quad, cv::Scalar(255));

                cv::Rect bbox = cv::boundingRect(quad);
                int xmin = std::max(0, bbox.x);
                int ymin = std::max(0, bbox.y);
                int xmax = std::min(width - 1, bbox.x + bbox.width);
                int ymax = std::min(height - 1, bbox.y + bbox.height);

                const uint8_t* color = tagColors[i % nColors];
                uint32_t rgbPacked = ((uint32_t)color[0] << 16) |
                                     ((uint32_t)color[1] << 8) |
                                     ((uint32_t)color[2]);
                float rgbFloat;
                std::memcpy(&rgbFloat, &rgbPacked, sizeof(float));

                for (int py = ymin; py <= ymax; py++) {
                    for (int px = xmin; px <= xmax; px++) {
                        if (cv::pointPolygonTest(quad, cv::Point2f(px, py), false) >= 0) {
                            Eigen::Vector3d pt = getPoint(px, py);
                            if (isValid(pt)) {
                                cloudPoints.push_back(pt.x());
                                cloudPoints.push_back(pt.y());
                                cloudPoints.push_back(pt.z());
                                cloudPoints.push_back(rgbFloat);
                            }
                        }
                    }
                }
            }

            auto segMsg = cv_bridge::CvImage(header, "mono8", segMask).toImageMsg();
            debugSegPub_->publish(*segMsg);

            sensor_msgs::msg::PointCloud2 cloudMsg;
            cloudMsg.header = header;
            cloudMsg.header.frame_id = camera_frame;
            int numPoints = cloudPoints.size() / 4;
            cloudMsg.height = 1;
            cloudMsg.width = numPoints;
            cloudMsg.is_dense = true;
            cloudMsg.is_bigendian = false;
            cloudMsg.point_step = 16;
            cloudMsg.row_step = cloudMsg.point_step * numPoints;

            sensor_msgs::msg::PointField fx, fy, fz, frgb;
            fx.name = "x"; fx.offset = 0; fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
            fy.name = "y"; fy.offset = 4; fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
            fz.name = "z"; fz.offset = 8; fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
            frgb.name = "rgb"; frgb.offset = 12; frgb.datatype = sensor_msgs::msg::PointField::FLOAT32; frgb.count = 1;
            cloudMsg.fields = {fx, fy, fz, frgb};

            cloudMsg.data.resize(cloudPoints.size() * sizeof(float));
            std::memcpy(cloudMsg.data.data(), cloudPoints.data(), cloudMsg.data.size());
            debugCloudPub_->publish(cloudMsg);
        }

        //Grayscale image
        if (is_debug_ && debugGrayPub_) {
            auto grayMsg = cv_bridge::CvImage(header, "mono8", matColorGray).toImageMsg();
            debugGrayPub_->publish(*grayMsg);
        }

        //Detection overlay
        bool shouldDraw = is_display_ || (is_debug_ && debugDetectionPub_);
        if (shouldDraw && !matColorBGR.empty()) {
            drawDetections(matColorBGR, detections, is_pose_detected);

            if (is_display_) {
                cv::imshow("color", matColorBGR);
            }
            if (is_debug_ && debugDetectionPub_) {
                cv::Mat matDetectionRGB;
                cv::cvtColor(matColorBGR, matDetectionRGB, cv::COLOR_BGR2RGB);
                auto detMsg = cv_bridge::CvImage(header, "rgb8", matDetectionRGB).toImageMsg();
                debugDetectionPub_->publish(*detMsg);
            }
        }
    }

    // ---- Main detection pipeline ----

    void processDetections(
        const cv::Mat& matColorGray,
        cv::Mat& matColorBGR,
        const std_msgs::msg::Header& header,
        std::function<Eigen::Vector3d(int, int)> getPoint,
        std::function<bool(const Eigen::Vector3d&)> isValid)
    {
        int width = matColorGray.cols;
        int height = matColorGray.rows;
        std::string camera_frame = header.frame_id;
        const double ransacThresh = 0.005;

        //Run AprilTag detection
        auto timeDetectionBegin = std::chrono::high_resolution_clock::now();
        image_u8_t image = {
            .width = matColorGray.cols,
            .height = matColorGray.rows,
            .stride = static_cast<int>(matColorGray.step),
            .buf = const_cast<uint8_t*>(matColorGray.data)
        };
        zarray_t* detections = apriltag_detector_detect(tagDetector_, &image);
        auto timeDetectionEnd = std::chrono::high_resolution_clock::now();

        //Look up publishing frame transform
        FrameTransform frameTf = lookupPublishingFrame(camera_frame);
        std::string pose_frame_id = frameTf.valid ? frameTf.frame_id : camera_frame;

        //Process each detected tag
        std::vector<bool> is_pose_detected;
        std::vector<CamTagPose> camTagPoses;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            //Filter by decision margin
            if (det->decision_margin < min_decision_margin_) {
                if (is_verbose_) {
                    std::cout << "Tag " << det->id
                        << " rejected: decision_margin=" << det->decision_margin
                        << " < " << min_decision_margin_ << std::endl;
                }
                is_pose_detected.push_back(false);
                continue;
            }

            //Bounds check
            bool outOfBounds = false;
            for (int j = 0; j < 4; j++) {
                if (det->p[j][0] < 0 || det->p[j][0] >= width ||
                    det->p[j][1] < 0 || det->p[j][1] >= height) {
                    outOfBounds = true;
                    break;
                }
            }
            if (outOfBounds || det->c[0] < 0 || det->c[0] >= width ||
                det->c[1] < 0 || det->c[1] >= height) {
                is_pose_detected.push_back(false);
                continue;
            }

            //Collect valid 3D points inside the tag quad
            auto tagPoints = collectTagPoints(det, width, height, getPoint, isValid);
            if (tagPoints.size() < 10) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Tag %d: only %zu valid depth points inside quad, need at least 10",
                    det->id, tagPoints.size());
                is_pose_detected.push_back(false);
                continue;
            }

            //RANSAC plane fitting
            PlaneResult plane = fitPlaneRANSAC(tagPoints, 100, ransacThresh);
            if (!plane.success) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Tag %d: RANSAC plane fit poor — %d/%zu inliers (%.0f%%)",
                    det->id, plane.inliers, tagPoints.size(), plane.inlierRatio * 100);
                is_pose_detected.push_back(false);
                continue;
            }

            //Compute tag position (median of inliers) and orientation
            Eigen::Vector3d pos0 = computeMedianInlierPosition(
                tagPoints, plane.normal, plane.d, ransacThresh);
            Eigen::Quaterniond quatCam = computeTagOrientation(
                det, plane.normal, plane.d, pos0, getPoint, isValid);

            camTagPoses.push_back({det->id, pos0, quatCam});

            //Transform to parent frame if needed
            Eigen::Vector3d posTag = pos0;
            Eigen::Quaterniond quatTag = quatCam;
            if (frameTf.valid) {
                quatTag = frameTf.rotation * quatCam;
                quatTag.normalize();
                posTag = frameTf.rotation * pos0 + frameTf.translation;
            }

            //Fix tag normal to a parent-frame axis (operates in publishing frame)
            if (!fix_normal_axis_.empty()) {
                quatTag = fixNormalAxis(quatTag);
            }

            publishTagPose(header, pose_frame_id, det->id, posTag, quatTag,
                det->decision_margin, static_cast<float>(plane.inlierRatio),
                static_cast<int>(tagPoints.size()));
            is_pose_detected.push_back(true);
        }

        //Publish TF frames
        if (publish_tf_ && !camTagPoses.empty()) {
            publishTagTransforms(header, camTagPoses);
        }

        //Debug visualizations
        publishDebugViz(detections, is_pose_detected, header, camera_frame,
            matColorGray, matColorBGR, width, height, getPoint, isValid);

        //Free detected tags
        apriltag_detections_destroy(detections);

        //Verbose timing output
        auto timeNow = std::chrono::high_resolution_clock::now();
        if (is_verbose_) {
            std::cout
                << "Process frame_index=" << frameIndex_
                << " tags=" << camTagPoses.size()
                << " detection="
                << (std::chrono::duration<double, std::milli>(
                       timeDetectionEnd - timeDetectionBegin)).count() << "ms"
                << " duration="
                << (std::chrono::duration<double, std::milli>(
                       timeNow - timeProcessLoop_)).count() << "ms"
                << std::endl;
        }
        timeProcessLoop_ = timeNow;
    }

    // ---- PointCloud mode callback ----

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloudMsg)
    {
        frameIndex_++;

        //Validate PointCloud2 is organized
        int width = cloudMsg->width;
        int height = cloudMsg->height;
        if (height <= 1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received unorganized point cloud (height=%d). "
                "An organized colored point cloud is required.", height);
            return;
        }

        //Find field offsets
        int offset_x = -1, offset_y = -1, offset_z = -1, offset_rgb = -1;
        for (const auto& field : cloudMsg->fields) {
            if (field.name == "x") offset_x = field.offset;
            else if (field.name == "y") offset_y = field.offset;
            else if (field.name == "z") offset_z = field.offset;
            else if (field.name == "rgb") offset_rgb = field.offset;
        }
        if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "PointCloud2 missing x/y/z fields");
            return;
        }
        if (offset_rgb < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "PointCloud2 missing rgb field. Use a colored point cloud topic.");
            return;
        }

        uint32_t point_step = cloudMsg->point_step;
        uint32_t row_step = cloudMsg->row_step;
        const uint8_t* cloudData = cloudMsg->data.data();

        //Extract grayscale image from point cloud RGB data
        cv::Mat matColorGray(height, width, CV_8UC1);
        cv::Mat invalidMask(height, width, CV_8UC1, cv::Scalar(0));
        cv::Mat matColorBGR;
        if (is_display_ || is_debug_) {
            matColorBGR = cv::Mat(height, width, CV_8UC3);
        }
        int invalidNan = 0, invalidZero = 0, invalidNearZero = 0;
        for (int py = 0; py < height; py++) {
            const uint8_t* rowPtr = cloudData + py * row_step;
            for (int px = 0; px < width; px++) {
                const uint8_t* ptPtr = rowPtr + px * point_step;
                float z = *reinterpret_cast<const float*>(ptPtr + offset_z);
                const uint8_t* rgbPtr = ptPtr + offset_rgb;
                uint8_t b = rgbPtr[0];
                uint8_t g = rgbPtr[1];
                uint8_t r = rgbPtr[2];
                if (!std::isfinite(z) || z < 1e-3) {
                    if (!std::isfinite(z)) invalidNan++;
                    else if (z == 0.0f) invalidZero++;
                    else invalidNearZero++;
                    matColorGray.at<uint8_t>(py, px) = 0;
                    invalidMask.at<uint8_t>(py, px) = 255;
                    if (is_display_ || is_debug_) {
                        matColorBGR.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 0);
                    }
                } else {
                    matColorGray.at<uint8_t>(py, px) =
                        static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
                    if (is_display_ || is_debug_) {
                        matColorBGR.at<cv::Vec3b>(py, px) = cv::Vec3b(b, g, r);
                    }
                }
            }
        }
        int totalInvalid = invalidNan + invalidZero + invalidNearZero;
        int totalPixels = width * height;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Invalid pixels: %d/%d (%.1f%%) — NaN: %d, zero: %d, near-zero: %d",
            totalInvalid, totalPixels,
            100.0 * totalInvalid / totalPixels,
            invalidNan, invalidZero, invalidNearZero);

        if (totalInvalid > 0) {
            cv::inpaint(matColorGray, invalidMask, matColorGray, 3, cv::INPAINT_TELEA);
            if (is_display_ || is_debug_) {
                cv::inpaint(matColorBGR, invalidMask, matColorBGR, 3, cv::INPAINT_TELEA);
            }
        }

        //3D point lookup from point cloud
        auto getPoint = [&](int px, int py) -> Eigen::Vector3d {
            const uint8_t* ptr = cloudData + py * row_step + px * point_step;
            float x = *reinterpret_cast<const float*>(ptr + offset_x);
            float y = *reinterpret_cast<const float*>(ptr + offset_y);
            float z = *reinterpret_cast<const float*>(ptr + offset_z);
            return Eigen::Vector3d(x, y, z);
        };

        auto isValid = [](const Eigen::Vector3d& p) -> bool {
            return std::isfinite(p.x()) && std::isfinite(p.y()) &&
                   std::isfinite(p.z()) && p.z() > 1e-3;
            // true;
        };

        processDetections(matColorGray, matColorBGR, cloudMsg->header, getPoint, isValid);
    }

    // ---- RGBD mode callback ----

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received first CameraInfo message");
        cameraInfo_ = msg;
    }

    void rgbdCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depthMsg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(),
            "Received first synced RGB+Depth pair: RGB %dx%d (%s), Depth %dx%d (%s)",
            imageMsg->width, imageMsg->height, imageMsg->encoding.c_str(),
            depthMsg->width, depthMsg->height, depthMsg->encoding.c_str());
        frameIndex_++;

        if (!cameraInfo_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No CameraInfo received yet, skipping frame");
            return;
        }

        // Convert RGB image to grayscale
        cv::Mat matColor;
        try {
            matColor = cv_bridge::toCvShare(imageMsg)->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge RGB error: %s", e.what());
            return;
        }

        cv::Mat matColorGray;
        if (matColor.channels() == 1) {
            matColorGray = matColor;
        } else {
            cv::cvtColor(matColor, matColorGray, cv::COLOR_RGB2GRAY);
        }

        // Convert depth image
        cv::Mat depthRaw;
        try {
            depthRaw = cv_bridge::toCvShare(depthMsg)->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge depth error: %s", e.what());
            return;
        }

        int width = matColorGray.cols;
        int height = matColorGray.rows;

        // Convert depth to float meters
        cv::Mat depthMeters;
        if (depthRaw.type() == CV_16UC1) {
            depthRaw.convertTo(depthMeters, CV_32FC1, depth_scale_);
        } else if (depthRaw.type() == CV_32FC1) {
            depthMeters = depthRaw;
        } else {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Unsupported depth format: %d", depthRaw.type());
            return;
        }

        cv::Mat matColorBGR;
        if (is_display_ || is_debug_) {
            if (matColor.channels() == 1) {
                cv::cvtColor(matColor, matColorBGR, cv::COLOR_GRAY2BGR);
            } else {
                cv::cvtColor(matColor, matColorBGR, cv::COLOR_RGB2BGR);
            }
        }

        // Camera intrinsics from CameraInfo
        double fx = cameraInfo_->k[0];
        double fy = cameraInfo_->k[4];
        double cx = cameraInfo_->k[2];
        double cy = cameraInfo_->k[5];

        // 3D point lookup using pinhole projection
        auto getPoint = [&](int px, int py) -> Eigen::Vector3d {
            float z = depthMeters.at<float>(py, px);
            double x = (px - cx) * z / fx;
            double y = (py - cy) * z / fy;
            return Eigen::Vector3d(x, y, z);
        };

        auto isValid = [](const Eigen::Vector3d& p) -> bool {
            return std::isfinite(p.x()) && std::isfinite(p.y()) &&
                   std::isfinite(p.z()) && p.z() > 1e-3;
            // return true;
        };

        processDetections(matColorGray, matColorBGR, imageMsg->header, getPoint, isValid);

        //Publish depth as point cloud
        if (is_debug_ && debugDepthCloudPub_) {
            std::vector<float> pts;
            for (int py = 0; py < height; py++) {
                for (int px = 0; px < width; px++) {
                    float z = depthMeters.at<float>(py, px);
                    if (!std::isfinite(z) || z < 1e-3) continue;
                    float x = static_cast<float>((px - cx) * z / fx);
                    float y = static_cast<float>((py - cy) * z / fy);
                    pts.push_back(x);
                    pts.push_back(y);
                    pts.push_back(z);
                }
            }

            int numPoints = pts.size() / 3;
            sensor_msgs::msg::PointCloud2 cloudMsg;
            cloudMsg.header = imageMsg->header;
            cloudMsg.height = 1;
            cloudMsg.width = numPoints;
            cloudMsg.is_dense = true;
            cloudMsg.is_bigendian = false;
            cloudMsg.point_step = 12;  // 3 floats: x, y, z
            cloudMsg.row_step = cloudMsg.point_step * numPoints;

            sensor_msgs::msg::PointField pfx, pfy, pfz;
            pfx.name = "x"; pfx.offset = 0; pfx.datatype = sensor_msgs::msg::PointField::FLOAT32; pfx.count = 1;
            pfy.name = "y"; pfy.offset = 4; pfy.datatype = sensor_msgs::msg::PointField::FLOAT32; pfy.count = 1;
            pfz.name = "z"; pfz.offset = 8; pfz.datatype = sensor_msgs::msg::PointField::FLOAT32; pfz.count = 1;
            cloudMsg.fields = {pfx, pfy, pfz};

            cloudMsg.data.resize(pts.size() * sizeof(float));
            std::memcpy(cloudMsg.data.data(), pts.data(), cloudMsg.data.size());
            debugDepthCloudPub_->publish(cloudMsg);
        }
    }

    // ---- TF publishing ----

    void publishTagTransforms(
        const std_msgs::msg::Header& header,
        const std::vector<CamTagPose>& camTagPoses)
    {
        //Optionally look up T_parent_cam once for all tags
        Eigen::Quaterniond q_pc = Eigen::Quaterniond::Identity();
        Eigen::Vector3d t_pc = Eigen::Vector3d::Zero();
        bool useParent = !publishing_frame_.empty() && tfBuffer_;

        if (useParent) {
            geometry_msgs::msg::TransformStamped parentToCam;
            try {
                parentToCam = tfBuffer_->lookupTransform(
                    publishing_frame_, header.frame_id,
                    tf2::TimePointZero,
                    tf2::durationFromSec(transform_timeout_));
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Could not look up transform %s -> %s: %s",
                    publishing_frame_.c_str(), header.frame_id.c_str(), ex.what());
                return;
            }
            q_pc = Eigen::Quaterniond(
                parentToCam.transform.rotation.w,
                parentToCam.transform.rotation.x,
                parentToCam.transform.rotation.y,
                parentToCam.transform.rotation.z);
            t_pc = Eigen::Vector3d(
                parentToCam.transform.translation.x,
                parentToCam.transform.translation.y,
                parentToCam.transform.translation.z);
        }

        //Publish one TF per detected tag
        for (const auto& tag : camTagPoses) {
            std::string child_frame = tag_frame_prefix_ + "_" + std::to_string(tag.id);

            Eigen::Quaterniond q_out = tag.orientation;
            Eigen::Vector3d t_out = tag.position;
            std::string parent_frame = header.frame_id;

            if (useParent) {
                //Compose: T_parent_tag = T_parent_cam * T_cam_tag
                q_out = q_pc * tag.orientation;
                q_out.normalize();
                t_out = q_pc * tag.position + t_pc;
                parent_frame = publishing_frame_;
            }

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = header.stamp;
            tf.header.frame_id = parent_frame;
            tf.child_frame_id = child_frame;
            tf.transform.translation.x = t_out.x();
            tf.transform.translation.y = t_out.y();
            tf.transform.translation.z = t_out.z();
            tf.transform.rotation.x = q_out.x();
            tf.transform.rotation.y = q_out.y();
            tf.transform.rotation.z = q_out.z();
            tf.transform.rotation.w = q_out.w();
            tfBroadcaster_->sendTransform(tf);
        }
    }

    // ---- Members ----

    //Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subCameraInfo_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subImage_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subDepth_;

    //RGBD synchronizer
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;

    //Cached camera info
    sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo_;

    //Publishers indexed by tag id
    std::map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> containerPub_;
    std::map<int, std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>> metricsPub_;

    //TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    //AprilTag detector
    apriltag_family_t* tagFamily_;
    apriltag_detector_t* tagDetector_;

    //TF lookup (for publishing_frame chaining)
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    //Parameters
    std::string input_mode_;
    bool is_verbose_;
    bool is_display_;
    bool publish_tf_;
    std::string tag_frame_prefix_;
    std::string publishing_frame_;
    double transform_timeout_;
    double depth_scale_;
    double min_decision_margin_;
    std::string fix_normal_axis_;
    Eigen::Vector3d fixedNormal_;
    bool is_debug_;

    //Debug publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debugCloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugSegPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugGrayPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugDetectionPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debugDepthCloudPub_;

    //State
    uint64_t frameIndex_;
    std::chrono::high_resolution_clock::time_point timeProcessLoop_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagNode>();

    if (node->isDisplay()) {
        //OpenCV HighGUI requires waitKey from the main thread
        rclcpp::Rate rate(30);
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            char key = cv::waitKey(1);
            if (key == 27) break;
            rate.sleep();
        }
    } else {
        rclcpp::spin(node);
    }

    rclcpp::shutdown();
    return 0;
}

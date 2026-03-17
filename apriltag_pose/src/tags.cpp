#include <iostream>
#include <map>
#include <cmath>
#include <cstring>
#include <chrono>
#include <functional>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
        this->declare_parameter("min_decision_margin", 30.0);
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
        is_debug_ = this->get_parameter("debug").as_bool();

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

    void processDetections(
        const cv::Mat& matColorGray,
        cv::Mat& matColorBGR,
        const std_msgs::msg::Header& header,
        std::function<Eigen::Vector3d(int, int)> getPoint,
        std::function<bool(const Eigen::Vector3d&)> isValid)
    {
        int width = matColorGray.cols;
        int height = matColorGray.rows;

        //Run AprilTag detection
        auto timeDetectionBegin = std::chrono::high_resolution_clock::now();
        image_u8_t image = {
            .width = matColorGray.cols,
            .height = matColorGray.rows,
            .stride = matColorGray.cols,
            .buf = const_cast<uint8_t*>(matColorGray.data)
        };
        zarray_t* detections = apriltag_detector_detect(tagDetector_, &image);
        auto timeDetectionEnd = std::chrono::high_resolution_clock::now();

        //Determine the frame for publishing
        std::string camera_frame = header.frame_id;
        Eigen::Quaterniond q_pc = Eigen::Quaterniond::Identity();
        Eigen::Vector3d t_pc = Eigen::Vector3d::Zero();
        bool usePublishingFrame = !publishing_frame_.empty() && tfBuffer_;
        std::string pose_frame_id = camera_frame;

        if (usePublishingFrame) {
            geometry_msgs::msg::TransformStamped tfStamped;
            try {
                tfStamped = tfBuffer_->lookupTransform(
                    publishing_frame_, camera_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(transform_timeout_));
                q_pc = Eigen::Quaterniond(
                    tfStamped.transform.rotation.w,
                    tfStamped.transform.rotation.x,
                    tfStamped.transform.rotation.y,
                    tfStamped.transform.rotation.z);
                t_pc = Eigen::Vector3d(
                    tfStamped.transform.translation.x,
                    tfStamped.transform.translation.y,
                    tfStamped.transform.translation.z);
                pose_frame_id = publishing_frame_;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Could not look up transform %s -> %s: %s. Publishing in camera frame.",
                    publishing_frame_.c_str(), camera_frame.c_str(), ex.what());
                usePublishingFrame = false;
            }
        }

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

            //Retrieve center and corners image coordinates
            Eigen::Vector2i uv0(det->c[0], det->c[1]);
            Eigen::Vector2i uv1(det->p[0][0], det->p[0][1]);
            Eigen::Vector2i uv2(det->p[1][0], det->p[1][1]);
            Eigen::Vector2i uv3(det->p[2][0], det->p[2][1]);
            Eigen::Vector2i uv4(det->p[3][0], det->p[3][1]);

            //Bounds check
            if (
                uv0.x() < 0 || uv0.x() >= width ||
                uv0.y() < 0 || uv0.y() >= height ||
                uv1.x() < 0 || uv1.x() >= width ||
                uv1.y() < 0 || uv1.y() >= height ||
                uv2.x() < 0 || uv2.x() >= width ||
                uv2.y() < 0 || uv2.y() >= height ||
                uv3.x() < 0 || uv3.x() >= width ||
                uv3.y() < 0 || uv3.y() >= height ||
                uv4.x() < 0 || uv4.x() >= width ||
                uv4.y() < 0 || uv4.y() >= height
            ) {
                is_pose_detected.push_back(false);
                continue;
            }

            //Look up 3D positions
            Eigen::Vector3d pos0 = getPoint(uv0.x(), uv0.y());
            Eigen::Vector3d pos1 = getPoint(uv1.x(), uv1.y());
            Eigen::Vector3d pos2 = getPoint(uv2.x(), uv2.y());
            Eigen::Vector3d pos3 = getPoint(uv3.x(), uv3.y());

            //Check for invalid points (NaN or zero depth)
            if (!isValid(pos0) || !isValid(pos1) ||
                !isValid(pos2) || !isValid(pos3)) {
                is_pose_detected.push_back(false);
                continue;
            }

            //Compute orientation in camera frame (before any coordinate transform)
            Eigen::Vector3d vectZ_cam = (pos1 - pos2).normalized();
            Eigen::Vector3d vectY_cam = -(pos3 - pos2).normalized();
            Eigen::Matrix3d rotCam = Eigen::Matrix3d::Identity();
            rotCam.col(0) = -vectY_cam.cross(vectZ_cam);
            rotCam.col(1) = -vectZ_cam;
            rotCam.col(2) = -vectY_cam;
            Eigen::Quaterniond quatCam(rotCam);
            quatCam.normalize();

            //Store camera-frame pose for TF computation
            camTagPoses.push_back({det->id, pos0, quatCam});

            //Transform to parent frame if requested
            Eigen::Vector3d posTag = pos0;
            Eigen::Quaterniond quatTag = quatCam;
            if (usePublishingFrame) {
                quatTag = q_pc * quatCam;
                quatTag.normalize();
                posTag = q_pc * pos0 + t_pc;
            }

            int indexTag = det->id;

            //Initialize publisher for a newly detected tag
            if (containerPub_.count(indexTag) == 0) {
                containerPub_[indexTag] = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "apriltag_pose/pose_tag_" + std::to_string(indexTag), 10);
            }

            //Publish tag pose message
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = header.stamp;
            msg.header.frame_id = pose_frame_id;
            msg.pose.position.x = posTag.x();
            msg.pose.position.y = posTag.y();
            msg.pose.position.z = posTag.z();
            msg.pose.orientation.x = quatTag.x();
            msg.pose.orientation.y = quatTag.y();
            msg.pose.orientation.z = quatTag.z();
            msg.pose.orientation.w = quatTag.w();
            containerPub_[indexTag]->publish(msg);

            is_pose_detected.push_back(true);
        }

        //Publish per-tag TF frames
        if (publish_tf_ && !camTagPoses.empty()) {
            publishTagTransforms(header, camTagPoses);
        }

        //Publish debug point cloud and segmentation image
        if (is_debug_ && zarray_size(detections) > 0) {
            //Tag colors for visualization (BGR order for cloud, but we store RGB)
            static const uint8_t tagColors[][3] = {
                {255, 0, 0}, {0, 255, 0}, {0, 0, 255},
                {255, 255, 0}, {0, 255, 255}, {255, 0, 255},
            };
            static const int nColors = 6;

            cv::Mat segMask(height, width, CV_8UC1, cv::Scalar(0));
            std::vector<float> cloudPoints;  // x,y,z,rgb packed

            for (int i = 0; i < zarray_size(detections); i++) {
                if (!is_pose_detected.at(i)) continue;

                apriltag_detection_t* det;
                zarray_get(detections, i, &det);

                //Build quad polygon from corners
                std::vector<cv::Point> quad(4);
                for (int j = 0; j < 4; j++) {
                    quad[j] = cv::Point((int)det->p[j][0], (int)det->p[j][1]);
                }

                //Fill segmentation mask
                cv::fillConvexPoly(segMask, quad, cv::Scalar(255));

                //Compute bounding box of the quad
                cv::Rect bbox = cv::boundingRect(quad);
                int xmin = std::max(0, bbox.x);
                int ymin = std::max(0, bbox.y);
                int xmax = std::min(width - 1, bbox.x + bbox.width);
                int ymax = std::min(height - 1, bbox.y + bbox.height);

                //Pick color for this tag
                const uint8_t* color = tagColors[i % nColors];
                uint32_t rgbPacked = ((uint32_t)color[0] << 16) |
                                     ((uint32_t)color[1] << 8) |
                                     ((uint32_t)color[2]);
                float rgbFloat;
                std::memcpy(&rgbFloat, &rgbPacked, sizeof(float));

                //Iterate pixels inside bounding box, test polygon membership
                std::vector<cv::Point2f> testPts;
                std::vector<std::pair<int,int>> validPixels;
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

            //Publish segmentation image
            std_msgs::msg::Header segHeader = header;
            auto segMsg = cv_bridge::CvImage(segHeader, "mono8", segMask).toImageMsg();
            debugSegPub_->publish(*segMsg);

            //Publish debug point cloud
            sensor_msgs::msg::PointCloud2 cloudMsg;
            cloudMsg.header = header;
            cloudMsg.header.frame_id = camera_frame;
            int numPoints = cloudPoints.size() / 4;
            cloudMsg.height = 1;
            cloudMsg.width = numPoints;
            cloudMsg.is_dense = true;
            cloudMsg.is_bigendian = false;
            cloudMsg.point_step = 16;  // 4 floats: x, y, z, rgb
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

        //Draw detected tags on color frame
        if (is_display_ && !matColorBGR.empty()) {
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
                            cv::circle(
                                matColorBGR,
                                cv::Point((int)uv2.x(), (int)uv2.y()),
                                3, cv::Scalar(255, 0, 255), -1);
                        }
                    }
                }
                //Tag corners
                cv::circle(
                    matColorBGR,
                    cv::Point(det->p[0][0], det->p[0][1]),
                    5, cv::Scalar(0, 0, 255), -1);
                cv::circle(
                    matColorBGR,
                    cv::Point(det->p[1][0], det->p[1][1]),
                    5, cv::Scalar(0, 255, 0), -1);
                cv::circle(
                    matColorBGR,
                    cv::Point(det->p[2][0], det->p[2][1]),
                    5, cv::Scalar(255, 0, 0), -1);
                cv::circle(
                    matColorBGR,
                    cv::Point(det->p[3][0], det->p[3][1]),
                    5, cv::Scalar(255, 255, 0), -1);
                //Tag center
                cv::circle(
                    matColorBGR,
                    cv::Point(det->c[0], det->c[1]),
                    5, cv::Scalar(0, 255, 255), -1);
            }
            cv::imshow("color", matColorBGR);
        }

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
        if (is_display_) {
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
                    if (is_display_) {
                        matColorBGR.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 0);
                    }
                } else {
                    matColorGray.at<uint8_t>(py, px) =
                        static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
                    if (is_display_) {
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
            if (is_display_) {
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
            cv::cvtColor(matColor, matColorGray, cv::COLOR_BGR2GRAY);
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
        if (is_display_) {
            if (matColor.channels() == 1) {
                cv::cvtColor(matColor, matColorBGR, cv::COLOR_GRAY2BGR);
            } else {
                matColorBGR = matColor.clone();
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
        };

        processDetections(matColorGray, matColorBGR, imageMsg->header, getPoint, isValid);
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
    bool is_debug_;

    //Debug publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debugCloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugSegPub_;

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

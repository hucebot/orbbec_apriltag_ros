#include <iostream>
#include <map>
#include <deque>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
}

//---------------------------------------------------------------------
// Pose filter interface — add new filter types by subclassing this
//---------------------------------------------------------------------
class PoseFilter {
public:
    virtual ~PoseFilter() = default;
    virtual void addPose(int tag_id,
                         const Eigen::Vector3d& pos,
                         const Eigen::Quaterniond& quat) = 0;
    virtual bool getFilteredPose(int tag_id,
                                 Eigen::Vector3d& pos,
                                 Eigen::Quaterniond& quat) const = 0;
};

//---------------------------------------------------------------------
// Sliding-window median filter (component-wise for position & quaternion)
//---------------------------------------------------------------------
class MedianPoseFilter : public PoseFilter {
public:
    explicit MedianPoseFilter(int window_size) : windowSize_(window_size) {}

    void addPose(int tag_id,
                 const Eigen::Vector3d& pos,
                 const Eigen::Quaterniond& quat) override
    {
        auto& buf = buffers_[tag_id];
        buf.push_back({pos, quat});
        if (static_cast<int>(buf.size()) > windowSize_) {
            buf.pop_front();
        }
    }

    bool getFilteredPose(int tag_id,
                         Eigen::Vector3d& pos,
                         Eigen::Quaterniond& quat) const override
    {
        auto it = buffers_.find(tag_id);
        if (it == buffers_.end() || it->second.empty()) return false;

        const auto& buf = it->second;

        //Position: component-wise median
        std::vector<double> xs, ys, zs;
        xs.reserve(buf.size());
        ys.reserve(buf.size());
        zs.reserve(buf.size());
        for (const auto& e : buf) {
            xs.push_back(e.position.x());
            ys.push_back(e.position.y());
            zs.push_back(e.position.z());
        }
        pos = Eigen::Vector3d(median(xs), median(ys), median(zs));

        //Orientation: align to same hemisphere, then component-wise median
        std::vector<double> qxs, qys, qzs, qws;
        qxs.reserve(buf.size());
        qys.reserve(buf.size());
        qzs.reserve(buf.size());
        qws.reserve(buf.size());
        const Eigen::Vector4d ref = buf.front().orientation.coeffs();
        for (const auto& e : buf) {
            Eigen::Vector4d q = e.orientation.coeffs();
            if (q.dot(ref) < 0) q = -q;
            qxs.push_back(q.x());
            qys.push_back(q.y());
            qzs.push_back(q.z());
            qws.push_back(q.w());
        }
        //Eigen::Quaterniond stores (x,y,z,w) in coeffs but constructor is (w,x,y,z)
        quat = Eigen::Quaterniond(median(qws), median(qxs), median(qys), median(qzs));
        quat.normalize();

        return true;
    }

private:
    struct PoseEntry {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

    static double median(std::vector<double> v) {
        size_t n = v.size();
        auto mid = v.begin() + n / 2;
        std::nth_element(v.begin(), mid, v.end());
        if (n % 2 == 1) return *mid;
        double upper = *mid;
        std::nth_element(v.begin(), v.begin() + n / 2 - 1, v.end());
        return (v[n / 2 - 1] + upper) / 2.0;
    }

    int windowSize_;
    mutable std::map<int, std::deque<PoseEntry>> buffers_;
};

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode()
        : Node("apriltag_pose"), frameIndex_(0)
    {
        //Declare and load parameters
        this->declare_parameter("verbose", false);
        this->declare_parameter("display", false);
        this->declare_parameter("cloud_topic", "/camera/camera/depth/color/points");
        this->declare_parameter("publish_tf", false);
        this->declare_parameter("tag_frame_prefix", "apriltag");
        this->declare_parameter("publishing_frame", "");
        this->declare_parameter("transform_timeout", 0.1);
        this->declare_parameter("filter_type", "none");
        this->declare_parameter("filter_window", 5);

        is_verbose_ = this->get_parameter("verbose").as_bool();
        is_display_ = this->get_parameter("display").as_bool();
        std::string cloud_topic = this->get_parameter("cloud_topic").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        tag_frame_prefix_ = this->get_parameter("tag_frame_prefix").as_string();
        publishing_frame_ = this->get_parameter("publishing_frame").as_string();
        transform_timeout_ = this->get_parameter("transform_timeout").as_double();
        std::string filter_type = this->get_parameter("filter_type").as_string();
        int filter_window = this->get_parameter("filter_window").as_int();

        //Create pose filter if requested
        if (filter_type == "median") {
            poseFilter_ = std::make_unique<MedianPoseFilter>(filter_window);
            RCLCPP_INFO(this->get_logger(),
                "Pose filter enabled: median (window=%d)", filter_window);
        } else if (filter_type != "none") {
            RCLCPP_WARN(this->get_logger(),
                "Unknown filter_type '%s', filtering disabled", filter_type.c_str());
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

        //Subscribe to colored point cloud only
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        qos_profile.depth = 10;
        subCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile)),
            std::bind(&AprilTagNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "AprilTag detector initialized. Subscribing to:");
        RCLCPP_INFO(this->get_logger(), "  PointCloud: %s", cloud_topic.c_str());

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

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloudMsg)
    {
        frameIndex_++;
        auto timeFrameProcess = std::chrono::high_resolution_clock::now();

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

        //Extract grayscale image from point cloud RGB data.
        //Pixels with invalid depth (NaN/zero z) may have garbage RGB,
        //which creates black holes that break AprilTag quad detection.
        //We mark those pixels and inpaint them from valid neighbors.
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
        //Inpaint invalid pixels so depth holes don't corrupt tag detection
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

        //Helper: look up 3D point at pixel (px, py)
        auto getPoint = [&](int px, int py) -> Eigen::Vector3d {
            const uint8_t* ptr = cloudData + py * row_step + px * point_step;
            float x = *reinterpret_cast<const float*>(ptr + offset_x);
            float y = *reinterpret_cast<const float*>(ptr + offset_y);
            float z = *reinterpret_cast<const float*>(ptr + offset_z);
            return Eigen::Vector3d(x, y, z);
        };

        //Helper: check if a 3D point is valid
        auto isValid = [](const Eigen::Vector3d& p) -> bool {
            return std::isfinite(p.x()) && std::isfinite(p.y()) &&
                   std::isfinite(p.z()) && p.z() > 1e-3;
        };

        //Run AprilTag detection
        auto timeDetectionBegin = std::chrono::high_resolution_clock::now();
        image_u8_t image = {
            .width = matColorGray.cols,
            .height = matColorGray.rows,
            .stride = matColorGray.cols,
            .buf = matColorGray.data
        };
        zarray_t* detections = apriltag_detector_detect(tagDetector_, &image);
        auto timeDetectionEnd = std::chrono::high_resolution_clock::now();

        //Determine the frame for publishing
        std::string camera_frame = cloudMsg->header.frame_id;
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

            //Look up 3D positions from point cloud (camera frame)
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
            msg.header.stamp = cloudMsg->header.stamp;
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

        //Filter poses if enabled, publish filtered poses, select poses for TF
        std::vector<CamTagPose> tfTagPoses;
        if (poseFilter_) {
            for (const auto& tag : camTagPoses) {
                poseFilter_->addPose(tag.id, tag.position, tag.orientation);
                Eigen::Vector3d filtPos;
                Eigen::Quaterniond filtQuat;
                if (poseFilter_->getFilteredPose(tag.id, filtPos, filtQuat)) {
                    tfTagPoses.push_back({tag.id, filtPos, filtQuat});

                    //Transform to parent frame if requested
                    Eigen::Vector3d pubPos = filtPos;
                    Eigen::Quaterniond pubQuat = filtQuat;
                    if (usePublishingFrame) {
                        pubQuat = q_pc * filtQuat;
                        pubQuat.normalize();
                        pubPos = q_pc * filtPos + t_pc;
                    }

                    //Initialize publisher for new filtered tag
                    if (filteredPub_.count(tag.id) == 0) {
                        filteredPub_[tag.id] = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                            "apriltag_pose/filtered_pose_tag_" + std::to_string(tag.id), 10);
                    }

                    geometry_msgs::msg::PoseStamped msg;
                    msg.header.stamp = cloudMsg->header.stamp;
                    msg.header.frame_id = pose_frame_id;
                    msg.pose.position.x = pubPos.x();
                    msg.pose.position.y = pubPos.y();
                    msg.pose.position.z = pubPos.z();
                    msg.pose.orientation.x = pubQuat.x();
                    msg.pose.orientation.y = pubQuat.y();
                    msg.pose.orientation.z = pubQuat.z();
                    msg.pose.orientation.w = pubQuat.w();
                    filteredPub_[tag.id]->publish(msg);
                }
            }
        } else {
            tfTagPoses = camTagPoses;
        }

        //Publish per-tag TF frames (from filtered poses when available)
        if (publish_tf_ && !tfTagPoses.empty()) {
            publishTagTransforms(cloudMsg->header, tfTagPoses);
        }

        //Draw detected tags on color frame
        if (is_display_) {
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
                       timeNow - timeFrameProcess)).count() << "ms"
                << " period="
                << (std::chrono::duration<double, std::milli>(
                       timeNow - timeProcessLoop_)).count() << "ms"
                << std::endl;
        }
        timeProcessLoop_ = timeNow;
    }

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

    //Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud_;

    //Publishers indexed by tag id
    std::map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> containerPub_;
    std::map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> filteredPub_;

    //Pose filter
    std::unique_ptr<PoseFilter> poseFilter_;

    //TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    //AprilTag detector
    apriltag_family_t* tagFamily_;
    apriltag_detector_t* tagDetector_;

    //TF lookup (for publishing_frame chaining)
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    //Parameters
    bool is_verbose_;
    bool is_display_;
    bool publish_tf_;
    std::string tag_frame_prefix_;
    std::string publishing_frame_;
    double transform_timeout_;

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

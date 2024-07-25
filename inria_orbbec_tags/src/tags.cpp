#include <iostream>
#include <map>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Error.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <realtime_tools/realtime_publisher.h>

extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
}

//Global parameters
//Color image dimensions
constexpr int width = 1280;
constexpr int height = 720;
//Verbose mode
bool is_verbose = false;
bool is_display = false;
//ROS parameters
std::string ros_master_ip = "127.0.0.1";
std::string this_node_ip = "127.0.0.1";

//Global variables
std::atomic<bool> isContinue(true);
std::atomic<bool> isInit(false);
std::mutex mutexColor;
std::mutex mutexDepth;
std::mutex mutexCloud;
uint8_t* dataColor;
uint16_t* dataDepth;
OBColorPoint* dataCloud;
float scaleDepth = 1.0;
uint64_t indexDepth = 0;
uint64_t timestampSystemInMs = 0;

static void mainProcessing()
{
    //initialize ROS
    if (is_verbose) {
        std::cout << "Initialization ROS" << std::endl;
        std::cout << "ros_master_ip: " << ros_master_ip << std::endl;
        std::cout << "this_node_ip: " << this_node_ip << std::endl;
    }
    std::map<std::string, std::string> remappingROSParams;
    remappingROSParams["__master"] = std::string("http://") + ros_master_ip + std::string(":11311/");
    remappingROSParams["__ip"] = this_node_ip;
    ros::init(remappingROSParams, "inria_orbbec_tags");
    ros::NodeHandle handle;
    //Publishers container indexed by tag id
    std::map<int,realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::PoseStamped>> containerPub;

    //Display using OpenCV
    if (is_display) {
        cv::namedWindow("color", cv::WINDOW_NORMAL|cv::WINDOW_KEEPRATIO);
        cv::resizeWindow("color", width, height);
        cv::namedWindow("depth", cv::WINDOW_NORMAL|cv::WINDOW_KEEPRATIO);
        cv::resizeWindow("depth", width, height);
        cv::setMouseCallback("color",
            [] (int event, int x, int y, int flags, void* userdata) {
                (void)userdata;
                (void)flags;
                if (event == cv::EVENT_LBUTTONDOWN) {
                    mutexDepth.lock();
                    float scale = scaleDepth;
                    mutexDepth.unlock();
                    mutexCloud.lock();
                    if (x >= 0 && x < width && y >= 0 && y < height) {
                        //Retrieve point cloud position 
                        Eigen::Vector3d pos(
                            dataCloud[width*y+x].x*scale/1000.0,
                            dataCloud[width*y+x].y*scale/1000.0,
                            dataCloud[width*y+x].z*scale/1000.0);
                        //Apply coordinate transformation
                        pos = Eigen::Vector3d(pos.z(), -pos.x(), -pos.y());
                        std::cout << "Pixel: " << x << " " << y 
                            << " Point: " <<  pos.transpose() << std::endl;
                    }
                    mutexCloud.unlock();
                }
            },
            nullptr);
    }

    //Initialize tag detector
    apriltag_family_t* tagFamily = tag36h11_create();
    apriltag_detector_t* tagDetector = apriltag_detector_create();
    apriltag_detector_add_family(tagDetector, tagFamily);
    //Setup tag detector configuration
    tagDetector->quad_decimate = 1.0;
    tagDetector->quad_sigma = 0.0;
    tagDetector->refine_edges = 1;
    tagDetector->decode_sharpening = 0.25;
    tagDetector->nthreads = 1;
    tagDetector->debug = 0;

    //Main processing and display loop
    cv::Mat matColorRaw;
    cv::Mat matColorBGR;
    cv::Mat matColorGray;
    cv::Mat matDepthRaw;
    cv::Mat matDepthProcess1;
    cv::Mat matDepthProcess2;
    cv::Mat matDepthBGR;
    uint64_t lastIndexFrame = 0;
    auto timeProcessLoop = std::chrono::high_resolution_clock::now();
    auto timeDetectionBegin = std::chrono::high_resolution_clock::now();
    auto timeDetectionEnd = std::chrono::high_resolution_clock::now();
    uint64_t lastTimestampFrame = 0;
    while (isContinue.load()) {
        if (!ros::ok()) {
            isContinue.store(false);
        }
        auto timeFrameProcess = std::chrono::high_resolution_clock::now();
        if (isInit.load()) {
            //Retrieve color frame
            mutexColor.lock();
            matColorRaw = cv::Mat(height, width, CV_8UC3, dataColor);
            cv::cvtColor(matColorRaw, matColorBGR, cv::COLOR_RGB2BGR);
            cv::cvtColor(matColorRaw, matColorGray, cv::COLOR_RGB2GRAY);
            mutexColor.unlock();
            //Compute tags detection
            timeDetectionBegin = std::chrono::high_resolution_clock::now();
            image_u8_t image = { 
                .width = matColorGray.cols,
                .height = matColorGray.rows,
                .stride = matColorGray.cols,
                .buf = matColorGray.data
            };
            zarray_t* detections = apriltag_detector_detect(tagDetector, &image);
            timeDetectionEnd = std::chrono::high_resolution_clock::now();
            //Retrieve depth frame
            mutexDepth.lock();
            matDepthRaw = cv::Mat(height, width, CV_16UC1, dataDepth);
            float scale = scaleDepth;
            uint64_t indexNow = indexDepth;
            lastTimestampFrame = timestampSystemInMs;
            cv::threshold(matDepthRaw, matDepthProcess1, 3000.0/scale, 0, cv::THRESH_TRUNC);
            mutexDepth.unlock();
            //Skip processing and publishing 
            //if depth frame is not updated
            if (indexNow <= lastIndexFrame) {
                continue;
            }
            //Store which tag is detected
            std::vector<bool> is_pose_detected;
            //Retrieve point cloud and compute 
            //detected tags pose estimation
            mutexCloud.lock();
            for (int i=0;i<zarray_size(detections);i++) {
                apriltag_detection_t* det;
                zarray_get(detections, i, &det);
                //Retrieve center and corners image coordinates
                Eigen::Vector2i uv0(det->c[0], det->c[1]);
                Eigen::Vector2i uv1(det->p[0][0], det->p[0][1]);
                Eigen::Vector2i uv2(det->p[1][0], det->p[1][1]);
                Eigen::Vector2i uv3(det->p[2][0], det->p[2][1]);
                Eigen::Vector2i uv4(det->p[3][0], det->p[3][1]);
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
                //Retrieve positions from point clouds
                Eigen::Vector3d pos0(
                    dataCloud[width*uv0.y()+uv0.x()].x*scale/1000.0,
                    dataCloud[width*uv0.y()+uv0.x()].y*scale/1000.0,
                    dataCloud[width*uv0.y()+uv0.x()].z*scale/1000.0);
                Eigen::Vector3d pos1(
                    dataCloud[width*uv1.y()+uv1.x()].x*scale/1000.0,
                    dataCloud[width*uv1.y()+uv1.x()].y*scale/1000.0,
                    dataCloud[width*uv1.y()+uv1.x()].z*scale/1000.0);
                Eigen::Vector3d pos2(
                    dataCloud[width*uv2.y()+uv2.x()].x*scale/1000.0,
                    dataCloud[width*uv2.y()+uv2.x()].y*scale/1000.0,
                    dataCloud[width*uv2.y()+uv2.x()].z*scale/1000.0);
                Eigen::Vector3d pos3(
                    dataCloud[width*uv3.y()+uv3.x()].x*scale/1000.0,
                    dataCloud[width*uv3.y()+uv3.x()].y*scale/1000.0,
                    dataCloud[width*uv3.y()+uv3.x()].z*scale/1000.0);
                if (
                    pos0.z() < 1e-3 ||
                    pos1.z() < 1e-3 ||
                    pos2.z() < 1e-3 ||
                    pos3.z() < 1e-3
                ) {
                    is_pose_detected.push_back(false);
                    continue;
                }
                //Apply coordinate transformation
                pos0 = Eigen::Vector3d(pos0.z(), -pos0.x(), -pos0.y());
                pos1 = Eigen::Vector3d(pos1.z(), -pos1.x(), -pos1.y());
                pos2 = Eigen::Vector3d(pos2.z(), -pos2.x(), -pos2.y());
                pos3 = Eigen::Vector3d(pos3.z(), -pos3.x(), -pos3.y());
                //Estimate orientation from 3d corner positions
                Eigen::Vector3d vectZ = (pos1-pos2).normalized();
                Eigen::Vector3d vectY = -(pos3-pos2).normalized();
                int indexTag = det->id;
                Eigen::Vector3d posTag = pos0;
                Eigen::Matrix3d rotTag = Eigen::Matrix3d::Identity();
                rotTag.col(0) = -vectY.cross(vectZ);
                rotTag.col(1) = -vectZ;
                rotTag.col(2) = -vectY;
                Eigen::Quaterniond quatTag(rotTag);
                quatTag.normalize();
                //Initialize publisher for a newly detected tag
                if (containerPub.count(indexTag) == 0) {
                    containerPub.insert(std::make_pair(
                        indexTag, 
                        realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::PoseStamped>()));
                    containerPub.at(indexTag).reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(
                        handle, "inria_orbbec_tags/pose_tag_"+std::to_string(indexTag), 10));
                }
                //Write tag pose message
                auto& pubPose = containerPub.at(indexTag);
                if (pubPose->trylock()) {
                    pubPose->msg_.header.seq = indexNow;
                    pubPose->msg_.header.stamp.sec = (lastTimestampFrame / 1000UL);
                    pubPose->msg_.header.stamp.nsec = (lastTimestampFrame % 1000UL) * 1000000UL;
                    pubPose->msg_.header.frame_id = "map";
                    pubPose->msg_.pose.position.x = posTag.x();
                    pubPose->msg_.pose.position.y = posTag.y();
                    pubPose->msg_.pose.position.z = posTag.z();
                    pubPose->msg_.pose.orientation.x = quatTag.x();
                    pubPose->msg_.pose.orientation.y = quatTag.y();
                    pubPose->msg_.pose.orientation.z = quatTag.z();
                    pubPose->msg_.pose.orientation.w = quatTag.w();
                }
                is_pose_detected.push_back(true);
            }
            mutexCloud.unlock();
            //Process depth frame for display
            matDepthProcess1.convertTo(matDepthProcess2, CV_8UC1, scale*255.0/3000.0);
            cv::applyColorMap(matDepthProcess2, matDepthBGR, cv::COLORMAP_JET);
            //Draw detected tags on color frame
            if (is_display) {
                for (int i=0;i<zarray_size(detections);i++) {
                    apriltag_detection_t* det;
                    zarray_get(detections, i, &det);
                    //Draw plane points if tag is detected
                    if (is_pose_detected.at(i)) {
                        Eigen::Matrix3d homography;
                        homography(0,0) = det->H->data[0];
                        homography(0,1) = det->H->data[1];
                        homography(0,2) = det->H->data[2];
                        homography(1,0) = det->H->data[3];
                        homography(1,1) = det->H->data[4];
                        homography(1,2) = det->H->data[5];
                        homography(2,0) = det->H->data[6];
                        homography(2,1) = det->H->data[7];
                        homography(2,2) = det->H->data[8];
                        for (double x=-1.0;x<=1.0;x+=1.0/4.0) {
                            for (double y=-1.0;y<=1.0;y+=1.0/4.0) {
                                Eigen::Vector3d uv1(x, y, 1.0);
                                Eigen::Vector3d uv2 = homography*uv1;
                                uv2 = uv2*(1.0/uv2.z());
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
            }
            //Free detected tags
            apriltag_detections_destroy(detections);
            //Publish pose message        
            for (auto& it : containerPub) {
                if (it.second->msg_.header.seq == indexNow) {
                    it.second->unlockAndPublish();
                }
            }
            lastIndexFrame = indexNow;
            //Display color frame
            if (is_display) {
                cv::imshow("color", matColorBGR);
            }
            //Display depth frame
            if (is_display) {
                cv::imshow("depth", matDepthBGR);
            }
        }
        auto timeNow = std::chrono::high_resolution_clock::now();
        if (is_verbose) {
            std::cout 
                << "Process frame_index=" << lastIndexFrame
                << " timestamp=" << lastTimestampFrame << "ms"
                << " detection=" << (std::chrono::duration<double, std::milli>(timeDetectionEnd - timeDetectionBegin)).count() << "ms" 
                << " duration=" << (std::chrono::duration<double, std::milli>(timeNow - timeFrameProcess)).count() << "ms"
                << " period=" << (std::chrono::duration<double, std::milli>(timeNow - timeProcessLoop)).count() << "ms" 
                << std::endl;
        }
        timeProcessLoop = timeNow;
        //Render
        if (is_display) {
            char key = cv::waitKey(30);
            if (key == 27) {
                isContinue.store(false);
            }
        }
    }

    //Deallocate tag detector
    apriltag_detector_destroy(tagDetector);
    tag36h11_destroy(tagFamily);

    //Close windows
    if (is_display) {
        cv::destroyWindow("color");
        cv::destroyWindow("depth");
    }
}

int main(int argc, char** argv)
{
    //Parse arguments
    is_display = false;
    is_verbose = false;
    ros_master_ip = "127.0.0.1";
    this_node_ip = "127.0.0.1";
    if (argc >= 6) {
        std::cout << "Usage error" << std::endl;
        std::cout << "Usage: inria_orbbec_tags is_display is_verbose ros_master_ip this_node_ip" << std::endl;
        return 1;
    }
    if (argc >= 2) {
        is_display = std::stoi(argv[1]);
    }
    if (argc >= 3) {
        is_verbose = std::stoi(argv[2]);
    }
    if (argc >= 4) {
        ros_master_ip = std::string(argv[3]);
    }
    if (argc >= 5) {
        this_node_ip = std::string(argv[4]);
    }

    //Print SDK version
    if (is_verbose) {
        std::cout << "SDK version: " 
            << ob::Version::getMajor() << "." 
            << ob::Version::getMinor() << "." 
            << ob::Version::getPatch() << ":"
            << ob::Version::getStageVersion() << std::endl;
    }

    //Set logger verbosity level
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    //Initialize context and list available devices
    ob::Context ctx;
    std::shared_ptr<ob::DeviceList> devicesList = ctx.queryDeviceList();
    if(devicesList == nullptr || devicesList->deviceCount() == 0) {
        std::cout << "Error no device found" << std::endl;
        return 1;
    }
    //Initialize first found device
    std::shared_ptr<ob::Device> device = devicesList->getDevice(0);
    if (device == nullptr) {
        std::cout << "Error opening device" << std::endl;
        return 1;
    }
    //Print device information
    std::shared_ptr<ob::DeviceInfo> deviceInfo = device->getDeviceInfo();
    if (deviceInfo == nullptr) {
        std::cout << "Error retrieving device information" << std::endl;
        return 1;
    }
    if (is_verbose) {
        std::cout << "Device: " << deviceInfo->name() << std::endl;
        std::cout << "Firmware version: " << deviceInfo->firmwareVersion() << std::endl;
        std::cout << "Serial number: " << deviceInfo->serialNumber() << std::endl;
    }

    //Initialize the pipeline
    ob::Pipeline pipe(device);

    //Setup streams configuration
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    //Color stream
    std::shared_ptr<ob::StreamProfileList> profilesColorList = pipe.getStreamProfileList(OB_SENSOR_COLOR);
    if (profilesColorList == nullptr) {
        std::cout << "Error retrieving color profiles" << std::endl;
        return 1;
    }
    std::shared_ptr<ob::VideoStreamProfile> profileColor = 
        profilesColorList->getVideoStreamProfile(width, height, OB_FORMAT_RGB, 30);
    if (profileColor == nullptr) {
        std::cout << "Error retrieving asked color profile" << std::endl;
        return 1;
    }
    config->enableStream(profileColor);
    //Depth stream
    std::shared_ptr<ob::StreamProfileList> profilesDepthList = pipe.getD2CDepthProfileList(profileColor, ALIGN_D2C_SW_MODE);
    if (profilesDepthList == nullptr) {
        std::cout << "Error retrieving depth profiles" << std::endl;
        return 1;
    }
    std::shared_ptr<ob::VideoStreamProfile> profileDepth = 
        profilesDepthList->getVideoStreamProfile(640, 576, OB_FORMAT_Y16, 30);
    if (profileDepth == nullptr) {
        std::cout << "Error retrieving asked depth profile" << std::endl;
        return 1;
    }
    config->enableStream(profileDepth);
    //Define software alignment mode such that
    //depth frame sizes matches color frame sizes
    config->setAlignMode(ALIGN_D2C_SW_MODE);

    //Start pipeline streaming
    pipe.start(config);

    //Allocate data
    dataColor = new uint8_t[3*width*height];
    dataDepth = new uint16_t[width*height];
    dataCloud = new OBColorPoint[width*height];

    //Start processing and rendering thread
    std::thread threadDisplay(mainProcessing);

    //Main capture loop
    auto timeCaptureLoop = std::chrono::high_resolution_clock::now();
    auto timeCloudBegin = std::chrono::high_resolution_clock::now();
    auto timeCloudEnd = std::chrono::high_resolution_clock::now();
    while (isContinue.load()) {
        //Request frames
        std::shared_ptr<ob::FrameSet> frameSet = pipe.waitForFrames(500);
        auto timeFrameProcess = std::chrono::high_resolution_clock::now();
        if(frameSet == nullptr || frameSet->frameCount() != 2) {
            //Skip frames during partial initialization
            continue;
        } else {
            //Set is ready for processing
            isInit.store(true);
            //Retrieve color and depth frames
            std::shared_ptr<ob::ColorFrame> frameColor = frameSet->colorFrame();
            std::shared_ptr<ob::DepthFrame> frameDepth = frameSet->depthFrame();
            if (frameColor != nullptr) {
                //Copy color frame for processing
                if (frameColor->dataSize() != 3*width*height) {
                    std::cout << "Error color frame size" << std::endl;
                    return 1;
                }
                mutexColor.lock();
                std::memcpy(dataColor, frameColor->data(), frameColor->dataSize());
                mutexColor.unlock();
            }
            if (frameDepth != nullptr) {
                //Copy depth frame for processing
                if (frameDepth->dataSize() != width*height*sizeof(uint16_t)) {
                    std::cout << "Error depth frame size" << std::endl;
                    return 1;
                }
                mutexDepth.lock();
                std::memcpy(dataDepth, frameDepth->data(), frameDepth->dataSize());
                scaleDepth = frameDepth->getValueScale();
                indexDepth = frameDepth->index();
                timestampSystemInMs = frameDepth->systemTimeStamp();
                mutexDepth.unlock();
                //Compute point cloud
                timeCloudBegin = std::chrono::high_resolution_clock::now();
                OBCameraParam cameraParam = pipe.getCameraParam();
                ob::PointCloudFilter filterPointCloud;
                filterPointCloud.setPositionDataScaled(scaleDepth);
                filterPointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                filterPointCloud.setCameraParam(cameraParam);
                filterPointCloud.setCoordinateSystem(OB_RIGHT_HAND_COORDINATE_SYSTEM);
                std::shared_ptr<ob::Frame> framePoint = filterPointCloud.process(frameSet);
                timeCloudEnd = std::chrono::high_resolution_clock::now();
                //Copy point cloud for processing
                if (framePoint->dataSize() != width*height*sizeof(OBColorPoint)) {
                    std::cout << "Error point frame size" << std::endl;
                    return 1;
                }
                mutexCloud.lock();
                std::memcpy(dataCloud, framePoint->data(), framePoint->dataSize());
                mutexCloud.unlock();
            }
        }
        auto timeNow = std::chrono::high_resolution_clock::now();
        if (is_verbose) {
            std::cout 
                << "Capture frame_index=" << indexDepth 
                << " timestamp=" << timestampSystemInMs << "ms"
                << " cloud=" << (std::chrono::duration<double, std::milli>(timeCloudEnd - timeCloudBegin)).count() << "ms" 
                << " duration=" << (std::chrono::duration<double, std::milli>(timeNow - timeFrameProcess)).count() << "ms"
                << " period=" << (std::chrono::duration<double, std::milli>(timeNow - timeCaptureLoop)).count() << "ms" 
                << std::endl;
        }
        timeCaptureLoop = timeNow;
    }

    //Stop pipeline
    pipe.stop();

    //Stop processing
    isContinue.store(false);
    threadDisplay.join();
    
    //Deallocate memory
    delete[] dataColor;
    delete[] dataDepth;
    delete[] dataCloud;
    
    return 0;
}


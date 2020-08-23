#include <System.h>
#include <Converter.h>
#include <image_feature_msgs/ImageFeatures.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "block_queue.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <signal.h>

class SLAMNode
{
public:
    SLAMNode();

    bool start();

    void mainLoop();

    void stop();

private:
    struct RGBDF {
        std_msgs::Header header;
        cv::Mat color;
        cv::Mat depth;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat local_desc;
        cv::Mat global_desc;
    };

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

    void imageCallback(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    void featureCallback(const image_feature_msgs::ImageFeaturesConstPtr &msg);

    void publishPose(const cv::Mat &Tcw, const std_msgs::Header &image_header);

    bool getTransform(tf::StampedTransform &transform, std::string from, std::string to, ros::Time stamp);

    message_filters::Subscriber<sensor_msgs::Image> sub_image_, sub_depth_;
    ros::Subscriber sub_feature_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;
    message_filters::Synchronizer<sync_policy> image_sync_;
    sensor_msgs::CameraInfoPtr camera_info_msg_;

    // buffers for un-synchronized messages
    // the size limit bounds increasement only in case of no message at all on one of the topics
    std::queue<RGBDF> image_queue_;
    std::queue<RGBDF> feature_queue_;
    const size_t unsync_queue_size_ = 30;

    // buffer for synchronized messages pending processing
    // its size can be specified with a runtime param
    BlockingQueue<RGBDF> work_queue_;

    ros::Publisher pub_pose_, pub_pose2d_, pub_pc_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    std::unique_ptr<DXSLAM::System> slam_;
    std::string p_image_topic_, p_depth_topic_, p_feature_topic_, p_camera_info_topic_;
    std::string p_ref_frame_, p_parent_frame_, p_child_frame_;
    std::string p_vocabulary_, p_config_;
    const double planar_tol_ = 0.1;
};

SLAMNode *slam_node;

void sigIntHandler(int sig)
{
   ros::shutdown();
   if (slam_node) slam_node->stop();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dxslam", ros::init_options::NoSigintHandler);
    slam_node = new SLAMNode();
    if (!slam_node->start()) return -1;
    signal(SIGINT, sigIntHandler);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    slam_node->mainLoop();
    return 0;
}

SLAMNode::SLAMNode() : image_sync_(sync_policy(10))
{
    ros::NodeHandle pnh("~");

    pnh.param("image_topic", p_image_topic_, std::string("/camera/color/image_raw"));
    pnh.param("depth_topic", p_depth_topic_, std::string("/camera/depth/image_raw"));
    pnh.param("feature_topic", p_feature_topic_, std::string("/camera/color/features"));
    pnh.param("camera_info_topic", p_camera_info_topic_, std::string("/camera/color/camera_info"));
    int p_queue_size;
    pnh.param("queue_size", p_queue_size, 10);
    work_queue_.set_queue_limit(p_queue_size);
    // Set ROS frames between which we should publish tf. Default: reference frame -> RGB image frame
    pnh.param("pub_tf_parent_frame", p_parent_frame_, std::string(""));
    pnh.param("pub_tf_child_frame", p_child_frame_, std::string(""));
    pnh.param("reference_frame", p_ref_frame_, std::string("vslam_origin"));

    pnh.param("vocabulary", p_vocabulary_, std::string(DXSLAM_PATH) + "/Vocabulary/super.fbow");
    pnh.param("config", p_config_, std::string(CONFIG_PATH) + "/openloris.yaml");
}

bool SLAMNode::start()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    auto exist = [](std::string filename) {
        std::ifstream fs(filename);
        if (!fs.good()) std::cerr << "File does not exist: " << filename << std::endl;
        return fs.good();
    };

    if (!exist(p_vocabulary_)) {
        p_vocabulary_ = std::string(DXSLAM_PATH) + "/Vocabulary/" + p_vocabulary_;
        if (!exist(p_vocabulary_)) return false;
    }
    if (!exist(p_config_)) {
        p_config_ = std::string(CONFIG_PATH) + '/' + p_config_;
        if (!exist(p_config_)) return false;
    }

    std::cout << "Vocabulary: " << p_vocabulary_ << "\n";
    std::cout << "Config: " << p_config_ << "\n";

    // copy config file
    std::string tmp_config_file("/tmp/slam-settings.yaml");
    {
        std::ifstream src(p_config_, std::ios::in | std::ios::binary);
        std::ofstream dst(tmp_config_file, std::ios::out | std::ios::binary);
        dst << src.rdbuf();
        src.close();
        dst.close();
    }

    // add camera info into the tmp config file if needed
    cv::FileStorage config_fs(p_config_.c_str(), cv::FileStorage::READ);
    if (config_fs["Camera.fromTopic"].real() > 0) {
        //sensor_msgs::CameraInfoPtr ptr;
        //void callback(sensor_msgs::CameraInfoPtr &ptr, const sensor_msgs::CameraInfoConstPtr &msg) { ptr = msg; };
        ros::Subscriber sub_camera_info = nh.subscribe(p_camera_info_topic_, 1, &SLAMNode::cameraInfoCallback, this);
        while (!camera_info_msg_) {
            ros::spinOnce();
            ROS_INFO_THROTTLE(3, "Waiting for camera info from %s", p_camera_info_topic_.c_str());
        }
        std::ofstream tmp_config(tmp_config_file, std::ios::app);
        tmp_config << std::endl;
        tmp_config << "Camera.fx: " << camera_info_msg_->K[0] << std::endl;
        tmp_config << "Camera.fy: " << camera_info_msg_->K[4] << std::endl;
        tmp_config << "Camera.cx: " << camera_info_msg_->K[2] << std::endl;
        tmp_config << "Camera.cy: " << camera_info_msg_->K[5] << std::endl;
        tmp_config << "Camera.k1: " << camera_info_msg_->D[0] << std::endl;
        tmp_config << "Camera.k2: " << camera_info_msg_->D[1] << std::endl;
        tmp_config << "Camera.p1: " << camera_info_msg_->D[2] << std::endl;
        tmp_config << "Camera.p2: " << camera_info_msg_->D[3] << std::endl;
        tmp_config << "Camera.k3: " << camera_info_msg_->D[4] << std::endl;
        tmp_config << "Camera.width: " << camera_info_msg_->width << std::endl;
        tmp_config << "Camera.height: " << camera_info_msg_->height << std::endl;
        tmp_config << "Camera.bf: " << config_fs["Camera.baseline"].real() * camera_info_msg_->K[0] << std::endl;
        tmp_config.close();
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    slam_.reset(new DXSLAM::System(p_vocabulary_, tmp_config_file, DXSLAM::System::RGBD, true));

    pub_pc_ = pnh.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
    pub_pose_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    pub_pose2d_ = pnh.advertise<geometry_msgs::PoseStamped>("pose2d", 1);

    sub_image_.subscribe(nh, p_image_topic_, 1000);
    sub_depth_.subscribe(nh, p_depth_topic_, 1000);
    sub_feature_ = nh.subscribe(p_feature_topic_, 10, &SLAMNode::featureCallback, this);
    image_sync_.connectInput(sub_image_, sub_depth_);
    image_sync_.registerCallback(boost::bind(&SLAMNode::imageCallback, this, _1, _2));
    return true;
}

void SLAMNode::stop()
{
    work_queue_.close();
    slam_->Shutdown();

    // Save camera trajectory
    //slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void SLAMNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    camera_info_msg_.reset(new sensor_msgs::CameraInfo(*msg));
}

void SLAMNode::imageCallback(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvCopy(msgD, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    RGBDF features;
    bool found = false;
    while (!feature_queue_.empty()) {
        features = feature_queue_.front();
        feature_queue_.pop();
        if (features.header.stamp == msgRGB->header.stamp) {
            found = true;
            break;
        } else if (features.header.stamp > msgRGB->header.stamp) {
            // missed the corresponding features
            return;
        }
    }
    if (found) {
        features.color = cv_ptrRGB->image;
        features.depth = cv_ptrD->image;
        work_queue_.push(features);
    } else {
        RGBDF data;
        data.header = msgRGB->header;
        data.color = cv_ptrRGB->image;
        data.depth = cv_ptrD->image;
        image_queue_.push(data);
        if (image_queue_.size() > unsync_queue_size_)
            image_queue_.pop();
    }
}

void SLAMNode::featureCallback(const image_feature_msgs::ImageFeaturesConstPtr &msg)
{
    // find the image with exact match of stamp
    RGBDF image;
    bool found = false;
    while (!image_queue_.empty()) {
        image = image_queue_.front();
        image_queue_.pop();
        if (image.header.stamp == msg->header.stamp) {
            found = true;
            break;
        } else if (image.header.stamp > msg->header.stamp) {
            // missed the image at given stamp
            ROS_WARN("Could not find matched image for feature at %lf", msg->header.stamp.toSec());
            return;
        }
    }
    if (!found) {
        // image not yet arrived, put features into queue
        image.header = msg->header;
    }

    // Copy the ros image message to cv::Mat.
    for (auto p: msg->keypoints) {
        cv::KeyPoint kp;
        kp.pt.x = p.x;
        kp.pt.y = p.y;
        kp.octave = 0;
        image.keypoints.push_back(kp);
    }
    image.local_desc.create(image.keypoints.size(), 256, CV_32F);
    int n_rows = image.local_desc.rows;
    int n_cols = image.local_desc.cols;
    for (int j = 0; j < n_rows; j++) {
        for (int i = 0; i < n_cols; i++) {
            image.local_desc.at<float>(j, i) = msg->descriptors.data[j * n_cols + i];
        }
    }
    image.global_desc = cv::Mat(msg->global_descriptor, CV_32F);

    if (found) {
        // invoke the worker
        work_queue_.push(image);
    } else {
        feature_queue_.push(image);
        if (feature_queue_.size() > unsync_queue_size_)
            feature_queue_.pop();
    }
}

void SLAMNode::mainLoop()
{
    while (ros::ok()) {
        RGBDF image;
        if (!work_queue_.pop(image)) break;
        cv::Mat Tcw = slam_->TrackRGBD(image.color, image.depth, image.header.stamp.toSec(), image.keypoints, image.local_desc, image.global_desc);
        if (Tcw.empty()) {
            ROS_ERROR("Empty pose result!");
            continue;
        }
        publishPose(Tcw, image.header);
    }
}

void SLAMNode::publishPose(const cv::Mat &Tcw, const std_msgs::Header &image_header)
{
    // Publish 3D pose
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    // x right, y down, z forward

    Eigen::Matrix<double,3,3> eigm = DXSLAM::Converter::toMatrix3d(Rwc);
    Eigen::Quaterniond eigq(eigm);

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = p_ref_frame_;
    msg.header.stamp = image_header.stamp;
    msg.pose.position.x = twc.at<float>(0);
    msg.pose.position.y = twc.at<float>(1);
    msg.pose.position.z = twc.at<float>(2);
    msg.pose.orientation.x = eigq.x();
    msg.pose.orientation.y = eigq.y();
    msg.pose.orientation.z = eigq.z();
    msg.pose.orientation.w = eigq.w();
    pub_pose_.publish(msg);

    // Publish tf
    tf::Transform ref_to_image;
    tf::StampedTransform parent_to_ref, child_to_image;
    tf::poseMsgToTF(msg.pose, ref_to_image);
    std::string image_frame = image_header.frame_id;
    std::string parent_frame = p_parent_frame_, child_frame = p_child_frame_;
    bool get_tf_succeed = true;
    if (p_parent_frame_.empty()) {
        parent_frame = p_ref_frame_;
        parent_to_ref.setIdentity();
    } else {
        get_tf_succeed &= getTransform(parent_to_ref, parent_frame, p_ref_frame_, msg.header.stamp);
    }
    if (p_child_frame_.empty()) {
        child_frame = image_frame;
        child_to_image.setIdentity();
    } else {
        get_tf_succeed &= getTransform(child_to_image, child_frame, image_frame, msg.header.stamp);
    }
    if (get_tf_succeed) {
        tf::Transform parent_to_child = tf::Transform(parent_to_ref * ref_to_image * child_to_image.inverse());
        tf_broadcaster_.sendTransform(tf::StampedTransform(parent_to_child, msg.header.stamp, parent_frame, child_frame));
    }

    // Publish 2D pose
    if (pub_pose2d_.getNumSubscribers() > 0) {
        // 1. transform into the parent frame (usually "map")
        geometry_msgs::PoseStamped pose2d;
        tf_listener_.transformPose(parent_frame, msg, pose2d);
        // 2. rotate from camera coordinate (right-down-forward) to ROS coordinate (forward-left-up)
        tf::Quaternion q2d(pose2d.pose.orientation.x, pose2d.pose.orientation.y, pose2d.pose.orientation.z, pose2d.pose.orientation.w);
        tf::Quaternion cam2map(0.5, -0.5, 0.5, 0.5);
        q2d *= cam2map;
        // 3. warn if the actual pose is not in the x-y plane
        if (std::abs(pose2d.pose.position.z) > planar_tol_)
            ROS_WARN("Non-planar position: (%lf, %lf, %lf)", pose2d.pose.position.x, pose2d.pose.position.y, pose2d.pose.position.z);
        if (std::abs(q2d[0]) > planar_tol_ || std::abs(q2d[1]) > planar_tol_)
            ROS_WARN("Non-planar orientation: (%lf, %lf, %lf, %lf)", q2d[0], q2d[1], q2d[2], q2d[3]);
        // 4. make the pose strictly in the x-y plane and publish it
        double norm_factor = 1. / std::sqrt(q2d[2] * q2d[2] + q2d[3] * q2d[3]);
        pose2d.pose.position.z = 0;
        pose2d.pose.orientation.x = 0;
        pose2d.pose.orientation.y = 0;
        pose2d.pose.orientation.z = q2d[2] * norm_factor;
        pose2d.pose.orientation.w = q2d[3] * norm_factor;
        pub_pose2d_.publish(pose2d);
    }

    // Publish map points
    if (pub_pc_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud cloud;
        cloud.header.frame_id = p_ref_frame_;
        std::vector<geometry_msgs::Point32> geo_points;
        std::vector<DXSLAM::MapPoint*> points = slam_->GetTrackedMapPoints();
        //cout << points.size() << endl;
        for (std::vector<int>::size_type i = 0; i != points.size(); i++) {
            if (points[i]) {
                cv::Mat coords = points[i]->GetWorldPos();
                geometry_msgs::Point32 pt;
                pt.x = coords.at<float>(0);
                pt.y = coords.at<float>(1);
                pt.z = coords.at<float>(2);
                geo_points.push_back(pt);
            } else {
            }
        }
        //cout << geo_points.size() << endl;
        cloud.points = geo_points;
        pub_pc_.publish(cloud);
    }
}

bool SLAMNode::getTransform(tf::StampedTransform &transform, std::string from, std::string to, ros::Time stamp)
{
    try {
        //tf_listener_.waitForTransform(from, to, stamp, ros::Duration(0.5));
        tf_listener_.lookupTransform(from, to, stamp, transform);
        return true;
    } catch (tf::TransformException &e) {
        ROS_ERROR("Failed to get transform from %s to %s: %s",
            from.c_str(), to.c_str(), e.what());
        return false;
    }
}

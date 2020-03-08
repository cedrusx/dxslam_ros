#include <System.h> // ORB_SLAM2
#include <Converter.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "hfnet_msgs/Hfnet.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>

class Maslam
{
public:
    Maslam();

    ~Maslam();

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD, const hfnet_msgs::HfnetConstPtr& msgHf);

    void publishPose(const cv::Mat &Tcw, const std_msgs::Header &image_header);

    bool getTransform(tf::StampedTransform &transform, std::string from, std::string to, ros::Time stamp);

    message_filters::Subscriber<sensor_msgs::Image> sub_image_, sub_depth_;
    message_filters::Subscriber<hfnet_msgs::Hfnet> sub_hfnet_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, hfnet_msgs::Hfnet> sync_policy;
    message_filters::Synchronizer<sync_policy> image_sync_;

    ros::Publisher pub_pose_, pub_pose2d_, pub_pc_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    std::unique_ptr<ORB_SLAM2::System> slam_;
    std::string p_image_topic_, p_depth_topic_;
    std::string p_ref_frame_, p_parent_frame_, p_child_frame_;
    std::string p_orb_vocabulary_, p_orb_settings_;
    const double planar_tol_ = 0.1;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maslam");
    Maslam maslam;
    ros::spin();
    return 0;
}

Maslam::Maslam() : image_sync_(sync_policy(10))
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("image_topic", p_image_topic_, std::string("/camera/color/image_raw"));
    pnh.param("depth_topic", p_depth_topic_, std::string("/camera/depth/image_raw"));
    // Set ROS frames between which we should publish tf. Default: reference frame -> RGB image frame
    pnh.param("pub_tf_parent_frame", p_parent_frame_, std::string(""));
    pnh.param("pub_tf_child_frame", p_child_frame_, std::string(""));
    pnh.param("reference_frame", p_ref_frame_, std::string("vslam_origin"));

    pnh.param("orb_vocabulary", p_orb_vocabulary_, std::string(ORB_SLAM2_PATH) + "/Vocabulary/super.fbow");
    pnh.param("orb_settings", p_orb_settings_, std::string(CONFIG_PATH) + "/rs435.yaml");

    std::cout << "ORB_SLAM2 vocabulary: " << p_orb_vocabulary_ << "\n";
    std::cout << "ORB_SLAM2 settings: " << p_orb_settings_ << "\n";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    slam_.reset(new ORB_SLAM2::System(p_orb_vocabulary_, p_orb_settings_, ORB_SLAM2::System::RGBD, true));

    pub_pc_ = pnh.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
    pub_pose_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    pub_pose2d_ = pnh.advertise<geometry_msgs::PoseStamped>("pose2d", 1);

    sub_image_.subscribe(nh, p_image_topic_, 1000);
    sub_depth_.subscribe(nh, p_depth_topic_, 1000);
    sub_hfnet_.subscribe(nh, "/features", 1000);
    image_sync_.connectInput(sub_image_, sub_depth_,sub_hfnet_ );
    image_sync_.registerCallback(boost::bind(&Maslam::imageCallback, this, _1, _2, _3));
}

Maslam::~Maslam()
{
    slam_->Shutdown();

    // Save camera trajectory
    slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void Maslam::imageCallback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const hfnet_msgs::HfnetConstPtr& msgHf)
{
    printf("got images\n");
    // Copy the ros image message to cv::Mat.
    std::vector<cv::KeyPoint> keypoints;
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msgHf->image, sensor_msgs::image_encodings::BGR8);
    for (auto it = msgHf->local_points.begin(); it != msgHf->local_points.end(); ++it) {
        cv::KeyPoint kp;
        kp.pt.x = it->x;
        kp.pt.y = it->y;
        kp.octave = 0;
        keypoints.push_back(kp);
    }
    cv::Mat local_desc;
    cv::Mat global_desc;
    local_desc.create(keypoints.size(), 256, CV_32F);
    int n_rows = local_desc.rows;
    int n_cols = local_desc.cols;
    for (int j = 0; j<n_rows; j++)
    {
        uchar* data = local_desc.ptr<uchar>(j);
        auto data_ = msgHf->local_desc[j];
        for (int i = 0; i<n_cols; i++)
        {
            data[i] = data_.data[i];
        }
    }
    global_desc = cv::Mat(msgHf->global_desc.data,CV_32F);
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(msgRGB,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvCopy(msgD, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(), keypoints, local_desc, global_desc);
    if (Tcw.empty()) {
            return;
    }

    publishPose(Tcw, msgRGB->header);
}

void Maslam::publishPose(const cv::Mat &Tcw, const std_msgs::Header &image_header)
{
    // Publish 3D pose
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    // x right, y down, z forward

    Eigen::Matrix<double,3,3> eigm = ORB_SLAM2::Converter::toMatrix3d(Rwc);
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


    // POINT CLOUD
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = p_ref_frame_;
    std::vector<geometry_msgs::Point32> geo_points;
    std::vector<ORB_SLAM2::MapPoint*> points = slam_->GetTrackedMapPoints();
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

bool Maslam::getTransform(tf::StampedTransform &transform, std::string from, std::string to, ros::Time stamp)
{
    try {
        //tf_listener_.waitForTransform(from, to, stamp, ros::Duration(0.5));
        tf_listener_.lookupTransform(from, to, stamp, transform);
        return true;
    } catch (tf::TransformException e) {
        ROS_ERROR("Failed to get transform from %s to %s: %s",
            from.c_str(), to.c_str(), e.what());
        return false;
    }
}

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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

class Maslam
{
public:
    Maslam(ORB_SLAM2::System* slam);

    void imageCallback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);

    void publishPose(const geometry_msgs::PoseStamped &msg);

    ros::Publisher pub_pose_, pub_pc_, pub_pose_1_;
    tf::TransformListener tf_;
    tf::TransformBroadcaster tfB_;

private:
    ORB_SLAM2::System* slam_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maslam");

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

    Maslam maslam(&SLAM);

    ros::NodeHandle nh;
    ros::NodeHandle lnh("~");

    maslam.pub_pc_ = lnh.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
    maslam.pub_pose_ = lnh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    maslam.pub_pose_1_ = lnh.advertise<geometry_msgs::PoseStamped>("pose1", 1);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&Maslam::imageCallback, &maslam, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

Maslam::Maslam(ORB_SLAM2::System* slam) : slam_(slam) {}

void Maslam::imageCallback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    if (Tcw.empty()) {
            return;
    }

    // Publish pose and tf
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    // x right, y down, z forward

    Eigen::Matrix<double,3,3> eigm = ORB_SLAM2::Converter::toMatrix3d(Rwc);
    Eigen::Quaterniond eigq(eigm);

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = msgRGB->header.stamp;
    msg.pose.position.x = twc.at<float>(0);
    msg.pose.position.y = twc.at<float>(1);
    msg.pose.position.z = twc.at<float>(2);
    msg.pose.orientation.x = eigq.x();
    msg.pose.orientation.y = eigq.y();
    msg.pose.orientation.z = eigq.z();
    msg.pose.orientation.w = eigq.w();
    pub_pose_.publish(msg);
    tf::Transform map_to_cam;
    tf::poseMsgToTF(msg.pose, map_to_cam);
    tfB_.sendTransform(tf::StampedTransform(map_to_cam, msg.header.stamp, msg.header.frame_id, msgRGB->header.frame_id));

    cv::Mat pose = Tcw;

    // CAMERA POSE

    if (pose.empty()) {
            return;
    }
    // transform into right handed camera frame
    tf::Matrix3x3 rh_cameraPose(  - pose.at<float>(0,0),   pose.at<float>(0,1),   pose.at<float>(0,2),
                                  - pose.at<float>(1,0),   pose.at<float>(1,1),   pose.at<float>(1,2),
                                    pose.at<float>(2,0), - pose.at<float>(2,1), - pose.at<float>(2,2));

    tf::Vector3 rh_cameraTranslation( pose.at<float>(0,3),pose.at<float>(1,3), - pose.at<float>(2,3) );
    std::cout << Tcw << std::endl;
    printf("a: %f %f %f\n", twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));
    printf("b: %f %f %f\n", pose.at<float>(0,3), pose.at<float>(1,3), pose.at<float>(2,3));

    //rotate 270deg about z and 270deg about x
    tf::Matrix3x3 rotation270degZX( 0, 0, 1,
                                   -1, 0, 0,
                                    0,-1, 0);

    //publish right handed, x forward, y right, z down (NED)

    tf::Quaternion q;
    rh_cameraPose.getRotation(q);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = rh_cameraTranslation[0];
    p.pose.position.y = rh_cameraTranslation[1];
    p.pose.position.z = rh_cameraTranslation[2];
    p.pose.orientation.x = q[0];
    p.pose.orientation.y = q[1];
    p.pose.orientation.z = q[2];
    p.pose.orientation.w = q[3];

    pub_pose_1_.publish(p);

    // POINT CLOUD
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = "map";
    std::vector<geometry_msgs::Point32> geo_points;
    std::vector<ORB_SLAM2::MapPoint*> points = slam_->GetTrackedMapPoints();
    cout << points.size() << endl;
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
    cout << geo_points.size() << endl;
    cloud.points = geo_points;
    pub_pc_.publish(cloud);




    // global left handed coordinate system
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                                               -1, 1,-1, 1,
                                                               -1,-1, 1, 1,
                                                                1, 1, 1, 1);

    //prev_pose * T = pose
    cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();


    // transform into global right handed coordinate system, publish in ROS
    tf::Matrix3x3 cameraRotation_rh(  - world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
                                  - world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
                                    world_lh.at<float>(2,0), - world_lh.at<float>(2,1), - world_lh.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                            0, 0, 1,
                                            1, 0, 0);

    tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;
    tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
    tfB_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_pose"));
}


void Maslam::publishPose(const geometry_msgs::PoseStamped &msg)
{
/*
    const Eigen::Vector3f &pose = msg->pose;
    poseMsgPub_.header.stamp = lastScanTime_;
    poseMsgPub_.pose.position.x = pose.x();
    poseMsgPub_.pose.position.y = pose.y();
    poseMsgPub_.pose.orientation.w = cos(pose.z() * 0.5f);
    poseMsgPub_.pose.orientation.z = sin(pose.z() * 0.5f);*/
    pub_pose_1_.publish(msg);

/*
    poseWithCovMsgPub_.header.stamp = lastScanTime_;
    poseWithCovMsgPub_.pose.pose = poseMsgPub_.pose;
    poseWithCovMsgPub_.pose.covariance[0] = msg->confidence;
    poseUpdatePublisher_.publish(poseWithCovMsgPub_);
*/
/*
    tf::Transform map_to_pose;
    tf::poseMsgToTF(msg.pose, map_to_pose);
    if (p_pub_map_odom_transform_)
    {
        tf::StampedTransform odom_to_base;
        try
        {
            tf_.waitForTransform(p_odom_frame_, p_base_frame_, msg.header.stamp, ros::Duration(0.5));
            tf_.lookupTransform(p_odom_frame_, p_base_frame_, msg.header.stamp, odom_to_base);
        }
        catch(tf::TransformException e)
        {
            ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
            odom_to_base.setIdentity();
        }
        tf::Transform map_to_odom = tf::Transform(map_to_pose * odom_to_base.inverse());
        tfB_.sendTransform(tf::StampedTransform(map_to_odom, msg.header.stamp, p_map_frame_, p_odom_frame_));
    }

    if (p_pub_map_scanmatch_transform_){
        tfB_.sendTransform(tf::StampedTransform(map_to_pose, msg.header.stamp, p_map_frame_, p_scanmatch_frame_));
    }*/
}

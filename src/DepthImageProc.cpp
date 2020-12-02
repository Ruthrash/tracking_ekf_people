#include "tracking_ekf_people/DepthImageProc.h"


DepthImageProc::DepthImageProc(ros::NodeHandle &node)
{
    depth_info_sub = node.subscribe("/pepper_robot/camera/depth/camera_info", 1, &DepthImageProc::DepthInfoCB, this);
    rgb_info_sub = node.subscribe("/pepper_robot/camera/front/camera_info", 1, &DepthImageProc::RGBInfoCB, this);
    depth_info.header.frame_id = "";
    rgb_info.header.frame_id = "";

}

void DepthImageProc::DepthInfoCB(const sensor_msgs::CameraInfo &depth_info_msg)
{
    depth_model_.fromCameraInfo(depth_info_msg);
    depth_info = depth_info_msg;
    std::cout<<depth_info.header.frame_id<<","<<depth_info_msg.header.frame_id<<"\n";
}

void DepthImageProc::RGBInfoCB(const sensor_msgs::CameraInfo &rgb_info_msg)
{
    rgb_model_.fromCameraInfo(rgb_info_msg);
    rgb_info = rgb_info_msg;
    std::cout<<rgb_info.header.frame_id<<"\n";
}
//void DepthImageProc::CreatePointCloud<uint16_t>(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg, const image_geometry::PinholeCameraModel& depth_model_, const darknet_ros_msgs::BoundingBox& bb, double range_max);
//void DepthImageProc::CreatePointCloud<float>(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg, const image_geometry::PinholeCameraModel& depth_model_, const darknet_ros_msgs::BoundingBox& bb, double range_max);
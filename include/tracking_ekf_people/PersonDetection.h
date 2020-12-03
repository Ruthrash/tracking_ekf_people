#ifndef PERSON_DETECTION_H
#define PERSON_DETECTION_H


#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include "tracking_ekf_people/DepthImageProc.h"
#include "tracking_ekf_people/Clustering.h"

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

class PersonDetection : public DepthImageProc, public Clustering
{

public:
    PersonDetection();
    ~PersonDetection();
    PersonDetection(ros::NodeHandle &node);

protected:
    ros::Publisher depth_reg_pub, depth_reg_info_pub;

private:

    image_transport::CameraPublisher pub_registered_;

    boost::shared_ptr<tf2_ros::TransformListener> tf_;
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> yolo_sync_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sync_sub; 

    typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    void SyncYOLODepthCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& depth_image);

    //template<typename T>//creates a pointcloud of the person within YOLO's bounding box and outputs after filtering out outliers using Euclidean clustering
    //void CreatePointCloud(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg, const image_geometry::PinholeCameraModel& depth_model_, const darknet_ros_msgs::BoundingBox& bb, double range_max);


    
};


#endif
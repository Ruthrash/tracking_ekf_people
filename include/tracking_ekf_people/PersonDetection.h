#ifndef PERSON_DETECTION_H
#define PERSON_DETECTION_H



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


#include <algorithm>
#include <limits>
#include "tracking_ekf_people/DepthImageProc.h"
#include "tracking_ekf_people/Clustering.h"


class PersonDetection : public DepthImageProc, public Clustering
{

public:
    PersonDetection();
    ~PersonDetection();
    PersonDetection(ros::NodeHandle &node);

protected:

private:
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> yolo_sync_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sync_sub; 

    typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    void SyncYOLODepthCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& depth_image);

    ros::Subscriber depth_info_sub; 
    void DepthInfoCB(const sensor_msgs::CameraInfo &depth_camera_info);
    image_geometry::PinholeCameraModel depth_model_;

    
};



#endif
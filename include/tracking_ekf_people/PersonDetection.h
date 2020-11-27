#ifndef PERSON_DETECTION_H
#define PERSON_DETECTION_H

#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <algorithm>
#include <limits>
#include "tracking_ekf_people/Clustering.h"


class PersonDetection : public Clustering
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

    template<typename T>
    void CreatePointCloud(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg, const image_geometry::PinholeCameraModel& depth_model_, const darknet_ros_msgs::BoundingBox& bb, double range_max = 0.0);

};


























// Encapsulate differences between processing float and uint16_t depths
template<typename T> struct DepthTraits {};

template<>
struct DepthTraits<uint16_t>
{
  static inline bool valid(uint16_t depth) { return depth != 0; }
  static inline float toMeters(uint16_t depth) { return depth * 0.001f; } // originally mm
  static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
  static inline void initializeBuffer(std::vector<uint8_t>& buffer) {} // Do nothing - already zero-filled
};

template<>
struct DepthTraits<float>
{
  static inline bool valid(float depth) { return std::isfinite(depth); }
  static inline float toMeters(float depth) { return depth; }
  static inline float fromMeters(float depth) { return depth; }

  static inline void initializeBuffer(std::vector<uint8_t>& buffer)
  {
    float* start = reinterpret_cast<float*>(&buffer[0]);
    float* end = reinterpret_cast<float*>(&buffer[0] + buffer.size());
    std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
  }
};


#endif
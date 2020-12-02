#ifndef DEPTH_IMAGE_PROC_H
#define DEPTH_IMAGE_PROC_H
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>


#include <algorithm>
#include <limits>
class DepthImageProc
{
public:
    DepthImageProc(){};
    ~DepthImageProc(){};

protected:
    template<typename T>
    void CreatePointCloud(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg, const image_geometry::PinholeCameraModel& depth_model_, const darknet_ros_msgs::BoundingBox& bb, double range_max);


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


// Handles float or uint16 depths
template<typename T>//creates a pointcloud of the person within YOLO's bounding box and outputs after filtering out outliers using Euclidean clustering
void DepthImageProc::CreatePointCloud(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg, const image_geometry::PinholeCameraModel& depth_model_, const darknet_ros_msgs::BoundingBox& bb, double range_max)
{
    // Use correct principal point from calibration
    float center_x = depth_model_.cx();
    float center_y = depth_model_.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = DepthTraits<T>::toMeters( T(1) );
    float constant_x = unit_scaling / depth_model_.fx();
    float constant_y = unit_scaling / depth_model_.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
    const T* avg_depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
    
    int row_step = depth_msg->step / sizeof(T);
    avg_depth_row = avg_depth_row + (int)(row_step*((bb.ymin + bb.ymax)/2));//*((bb.ymin + bb.ymax)/2);
    std::cout<<"boxes:"<<bb.xmin<<", "<<bb.xmax<<", "<<bb.ymin<<", "<<bb.ymax<<"\n";
    std::cout<<"iter: "<<(int)cloud_msg->height<<", "<<(int)cloud_msg->width<<"\n";
    for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
    //for (int v = (int)bb.ymin; v <= (int)bb.ymax; ++v, depth_row += row_step)
    {
        for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
        //for (int u = (int)bb.xmin; u <= (int)bb.xmax; ++u, ++iter_x, ++iter_y, ++iter_z)
        {
        T depth = depth_row[u];

        // Missing points denoted by NaNs
        if (!DepthTraits<T>::valid(depth))
        {
            if (range_max != 0.0)
            {
                depth = DepthTraits<T>::fromMeters(range_max);
            }
            else
            {
                *iter_x = *iter_y = *iter_z = bad_point;
                continue;
            }
        }
        // Fill in XYZ
        if(u>=bb.xmin && u<=bb.xmax && v>=bb.ymin && v<=bb.ymax)
        {
            //if(DepthTraits<T>::toMeters(depth) <= 1.2*avg_depth_row[(int)((bb.xmin + bb.xmax)/2)]     )
            //{
                *iter_x = (u - center_x) * depth * constant_x;
                *iter_y = (v - center_y) * depth * constant_y;
                *iter_z = DepthTraits<T>::toMeters(depth);
            //}
        }
        }
    }
}

#endif 
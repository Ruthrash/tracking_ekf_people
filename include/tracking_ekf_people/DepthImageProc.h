#ifndef DEPTH_IMAGE_PROC_H
#define DEPTH_IMAGE_PROC_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

//#include <

#include <algorithm>
#include <limits>
class DepthImageProc
{
public:
    DepthImageProc(){};
    ~DepthImageProc(){};
    DepthImageProc(ros::NodeHandle &node);

protected:
    sensor_msgs::CameraInfo rgb_info, depth_info;
    image_geometry::PinholeCameraModel depth_model_;
    image_geometry::PinholeCameraModel rgb_model_;
    template<typename T>
    void CreatePointCloud(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg, const image_geometry::PinholeCameraModel& depth_model_, const darknet_ros_msgs::BoundingBox& bb, double range_max);

    template<typename T>
    void RegisterDepthToRGB(const sensor_msgs::ImageConstPtr& depth_msg,
                              const sensor_msgs::ImagePtr& registered_msg,
                              const Eigen::Affine3d& depth_to_rgb);

private:
    ros::Subscriber depth_info_sub; 
    void DepthInfoCB(const sensor_msgs::CameraInfo &depth_camera_info);
    ros::Subscriber rgb_info_sub; 
    void RGBInfoCB(const sensor_msgs::CameraInfo &rgb_camera_info);                              

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



template<typename T>
void DepthImageProc::RegisterDepthToRGB(const sensor_msgs::ImageConstPtr& depth_msg,
                              const sensor_msgs::ImagePtr& registered_msg,
                              const Eigen::Affine3d& depth_to_rgb)
{
  // Allocate memory for registered depth image
  registered_msg->step = registered_msg->width * sizeof(T);
  registered_msg->data.resize( registered_msg->height * registered_msg->step );
  // data is already zero-filled in the uint16 case, but for floats we want to initialize everything to NaN.
  DepthTraits<T>::initializeBuffer(registered_msg->data);

  // Extract all the parameters we need
  double inv_depth_fx = 1.0 / depth_model_.fx();
  double inv_depth_fy = 1.0 / depth_model_.fy();
  double depth_cx = depth_model_.cx(), depth_cy = depth_model_.cy();
  double depth_Tx = depth_model_.Tx(), depth_Ty = depth_model_.Ty();
  double rgb_fx = rgb_model_.fx(), rgb_fy = rgb_model_.fy();
  double rgb_cx = rgb_model_.cx(), rgb_cy = rgb_model_.cy();
  //double rgb_fx = 2740139509 , rgb_fy = 275.74184670;
  //double rgb_cx = 141.1319308, rgb_cy = 106.39174605;
  double rgb_Tx = rgb_model_.Tx(), rgb_Ty = rgb_model_.Ty();
  //std::cout<<rgb_fx<<", "rgb_fy<<", "<<rgb_Tx<<", "<<rgb_Ty<<", "rgb_cx<<", "<<rgb_cy<<std::endl;
  
  // Transform the depth values into the RGB frame
  /// @todo When RGB is higher res, interpolate by rasterizing depth triangles onto the registered image  
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  T* registered_data = reinterpret_cast<T*>(&registered_msg->data[0]);
  int raw_index = 0;
  for (unsigned v = 0; v < depth_msg->height; ++v, depth_row += row_step)
  {
    for (unsigned u = 0; u < depth_msg->width; ++u, ++raw_index)
    {
      T raw_depth = depth_row[u];
      if (!DepthTraits<T>::valid(raw_depth))
        continue;
      
      double depth = DepthTraits<T>::toMeters(raw_depth);

      /// @todo Combine all operations into one matrix multiply on (u,v,d)
      // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
      Eigen::Vector4d xyz_depth;
      xyz_depth << ((u - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                   ((v - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                   depth,
                   1;

      // Transform to RGB camera frame
      Eigen::Vector4d xyz_rgb = depth_to_rgb * xyz_depth;

      // Project to (u,v) in RGB image
      double inv_Z = 1.0 / xyz_rgb.z();
      int u_rgb = (rgb_fx*xyz_rgb.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
      int v_rgb = (rgb_fy*xyz_rgb.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;
      
      if (u_rgb < 0 || u_rgb >= (int)registered_msg->width ||
          v_rgb < 0 || v_rgb >= (int)registered_msg->height)
        continue;
      
      T& reg_depth = registered_data[v_rgb*registered_msg->width + u_rgb];
      T  new_depth = DepthTraits<T>::fromMeters(xyz_rgb.z());
      // Validity and Z-buffer checks
      if (!DepthTraits<T>::valid(reg_depth) || reg_depth > new_depth)
        reg_depth = new_depth;
    }
  }
}




#endif 
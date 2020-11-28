#include "tracking_ekf_people/DepthImageProc.h"


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
    int row_step = depth_msg->step / sizeof(T);
    std::cout<<"boxes:"<<bb.xmin<<", "<<bb.xmax<<", "<<bb.ymin<<", "<<bb.ymax<<"\n";
    std::cout<<"iter: "<<(int)cloud_msg->height<<", "<<(int)cloud_msg->width<<"\n";
    //for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
    for (int v = (int)bb.ymin; v <= (int)bb.ymax; ++v, depth_row += row_step)
    {
        //for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
        for (int u = (int)bb.xmin; u <= (int)bb.xmax; ++u, ++iter_x, ++iter_y, ++iter_z)
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
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = DepthTraits<T>::toMeters(depth);
        }
    }
}
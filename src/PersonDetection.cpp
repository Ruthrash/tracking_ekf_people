#include "tracking_ekf_people/PersonDetection.h"

PersonDetection::PersonDetection(){}
PersonDetection::~PersonDetection(){}

PersonDetection::PersonDetection(ros::NodeHandle &node) : Clustering(node)
{
    yolo_sync_sub.subscribe(node, "/darknet_ros/bounding_boxes",1);
    depth_sync_sub.subscribe(node, "/pepper_robot/camera/depth_registered/image_rect",1);
    std::cout<<"running corrections \n";
    sync_.reset(new Sync(MySyncPolicy(10), yolo_sync_sub, depth_sync_sub));
    sync_->registerCallback(boost::bind(&PersonDetection::SyncYOLODepthCB, this, _1, _2));
    depth_info_sub = node.subscribe("/pepper_robot/camera/depth/camera_info", 1, &PersonDetection::DepthInfoCB, this);
}

void PersonDetection::DepthInfoCB(const sensor_msgs::CameraInfo &depth_info_msg)
{
    depth_model_.fromCameraInfo(depth_info_msg);
}


void PersonDetection::SyncYOLODepthCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& depth_msg)
{
    visualization_msgs::MarkerArray marker_array;
    std::cout<<"running corrections \n";
    for (int i = 0; i < bb->bounding_boxes.size(); ++i)
    {
        if(bb->bounding_boxes[i].Class == "person")
        {
            std::cout<<"persons\n";
        
            //get each person detection as a XYZ pointcloud
            sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
            cloud_msg->header = depth_msg->header;
            cloud_msg->height = depth_msg->height;
            cloud_msg->width  = depth_msg->width;
            cloud_msg->is_dense = false;
            cloud_msg->is_bigendian = false;
            sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
            pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
            darknet_ros_msgs::BoundingBoxes boxes = *bb;

            if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
            {
                CreatePointCloud<uint16_t>(depth_msg, cloud_msg, depth_model_, boxes.bounding_boxes[i], 5.0);
            }
            else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            {
                CreatePointCloud<float>(depth_msg, cloud_msg, depth_model_, boxes.bounding_boxes[i], 5.0);
            }
            else
            {
                ROS_INFO("Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
                return;
            }
            marker_array.markers.push_back(Clustering::GetPersonBoundingBoxes(cloud_msg));
        }
    }
    Clustering::PublishBoxesArray(marker_array);
}




// Handles float or uint16 depths
template<typename T>
void PersonDetection::CreatePointCloud(const sensor_msgs::ImageConstPtr& depth_msg, sensor_msgs::PointCloud2::Ptr& cloud_msg, const image_geometry::PinholeCameraModel& depth_model_, const darknet_ros_msgs::BoundingBox& bb, double range_max)
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
    for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
    {
        for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
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






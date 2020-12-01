#include "tracking_ekf_people/PersonDetection.h"

PersonDetection::PersonDetection(){}
PersonDetection::~PersonDetection(){}

PersonDetection::PersonDetection(ros::NodeHandle &node) : Clustering(node)
{
    yolo_sync_sub.subscribe(node, "/darknet_ros/bounding_boxes",300);
    depth_sync_sub.subscribe(node, "/pepper_robot/camera/depth_registered/image_rect",300);
    std::cout<<"running corrections \n";
    sync_.reset(new Sync(MySyncPolicy(3000), yolo_sync_sub, depth_sync_sub));
    sync_->registerCallback(boost::bind(&PersonDetection::SyncYOLODepthCB, this, _1, _2));
    depth_info_sub = node.subscribe("/pepper_robot/camera/depth_registered/camera_info", 1, &PersonDetection::DepthInfoCB, this);
}

void PersonDetection::DepthInfoCB(const sensor_msgs::CameraInfo &depth_info_msg)
{
    depth_model_.fromCameraInfo(depth_info_msg);
}


void PersonDetection::SyncYOLODepthCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& depth_msg)
{
    visualization_msgs::MarkerArray marker_array;
    std::cout<<"running corrections \n";
    int count = 0;
    for (int i = 0; i < bb->bounding_boxes.size(); ++i)
    {
        if(bb->bounding_boxes[i].Class == "person")
        {
            count++;        
            //get each person detection as a XYZ pointcloud
            sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
            std::cout<<"frame id depth "<<depth_msg->header.frame_id<<"\n";
            cloud_msg->header = depth_msg->header;
            //cloud_msg->height = bb->bounding_boxes[i].ymax - bb->bounding_boxes[i].ymin;
            cloud_msg->height = depth_msg->height;
            //cloud_msg->width  = bb->bounding_boxes[i].xmax - bb->bounding_boxes[i].xmin;
            cloud_msg->width  = depth_msg->width;
            cloud_msg->is_dense = false;
            cloud_msg->is_bigendian = false;
            sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
            pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
            darknet_ros_msgs::BoundingBoxes boxes = *bb;

            if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
            {
                CreatePointCloud<uint16_t>(depth_msg, cloud_msg, depth_model_, boxes.bounding_boxes[i], 0.0);
            }
            else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            {
                CreatePointCloud<float>(depth_msg, cloud_msg, depth_model_, boxes.bounding_boxes[i], 0.0);
            }
            else
            {
                ROS_INFO("Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
                return;
            }
            visualization_msgs::Marker marker_ = Clustering::GetPersonBoundingBoxes(cloud_msg, i);
            //for(int j = 0; j < marker_array_d.markers.size() ; j++)
             //   marker_array.markers.push_back(marker_array_d.markers[j]);
            //Clustering::PersonCloud(*cloud_msg);
            //ros::Duration(0.5).sleep();
            marker_array.markers.push_back(marker_);
           // marker_array.header = marker_.header;
        }
    }
    Clustering::PublishBoxesArray(marker_array);
    std::cout<<marker_array.markers.size()<<"vountttt\n";
  
}



// Handles float or uint16 depths
template<typename T>//creates a pointcloud of the person within YOLO's bounding box and outputs after filtering out outliers using Euclidean clustering
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
            *iter_x = (u - center_x) * depth * constant_x;
            *iter_y = (v - center_y) * depth * constant_y;
            *iter_z = DepthTraits<T>::toMeters(depth);
        }
        }
    }
}

/*
template<typename T>
void RegisterNodelet::convert(const sensor_msgs::ImageConstPtr& depth_msg,
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
}*/



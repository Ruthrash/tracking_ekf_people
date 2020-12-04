#include "tracking_ekf_people/PersonDetection.h"

PersonDetection::PersonDetection(){}
PersonDetection::~PersonDetection(){}

PersonDetection::PersonDetection(ros::NodeHandle &node) : Clustering(node), DepthImageProc(node)
{
    yolo_sync_sub.subscribe(node, "/darknet_ros/bounding_boxes",300);
    //depth_sync_sub.subscribe(node, "/pepper_robot/camera/depth/image_raw",300);
    depth_sync_sub.subscribe(node, "/camera/depth/image",300);
    sync_.reset(new Sync(MySyncPolicy(3000), yolo_sync_sub, depth_sync_sub));
    sync_->registerCallback(boost::bind(&PersonDetection::SyncYOLODepthCB, this, _1, _2));
    //depth_info_sub = node.subscribe("/camera/depth_registered/camera_info", 1, &PersonDetection::DepthInfoCB, this);
    tf_buffer_.reset( new tf2_ros::Buffer );
    tf_.reset( new tf2_ros::TransformListener(*tf_buffer_) );
    depth_reg_pub = node.advertise<sensor_msgs::Image>("image_rect", 1000);
    depth_reg_info_pub = node.advertise<sensor_msgs::CameraInfo>("image_rect/camera_info", 1000);
}




void PersonDetection::SyncYOLODepthCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& depth_msg)
{
    ros::Time start = ros::Time::now();
    visualization_msgs::MarkerArray marker_array;
    std::cout<<"running corrections \n";
    int count = 0;
    for (int i = 0; i < bb->bounding_boxes.size(); ++i)
    {
        if(bb->bounding_boxes[i].Class == "person" && rgb_info.header.frame_id != ""&& depth_info.header.frame_id != "")
        { 
            
            count++;        
            Eigen::Affine3d depth_to_rgb;
            try
            {
                geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform (
                                    rgb_info.header.frame_id, depth_info.header.frame_id,
                                    depth_msg->header.stamp);

                tf::transformMsgToEigen(transform.transform, depth_to_rgb);
            }
            catch (tf2::TransformException& ex)
            {
                ROS_INFO( "TF2 exception:\n%s", ex.what());
                return;
                /// @todo Can take on order of a minute to register a disconnect callback when we
                /// don't call publish() in this cb. What's going on roscpp?
            }
            // Allocate registered depth image
            sensor_msgs::ImagePtr registered_msg( new sensor_msgs::Image );
            registered_msg->header.stamp    = depth_msg->header.stamp;
            registered_msg->header.frame_id = rgb_info.header.frame_id;
            registered_msg->encoding        = depth_msg->encoding;
            
            cv::Size resolution = rgb_model_.reducedResolution();
            registered_msg->height = resolution.height;
            registered_msg->width  = resolution.width;
            // step and data set in convert(), depend on depth data type

            //get each person detection as a XYZ pointcloud
            sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
            cloud_msg->header = registered_msg->header; cloud_msg->height = registered_msg->height; cloud_msg->width  = registered_msg->width;
            cloud_msg->is_dense = false; cloud_msg->is_bigendian = false;
            sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg); pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

            if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
            {
                //RegisterDepthToRGB<uint16_t>(depth_msg, registered_msg, depth_to_rgb);
                CreatePointCloud<uint16_t>(depth_msg, cloud_msg, rgb_model_, bb->bounding_boxes[i], 0.0);                
            }
            else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            {
                //RegisterDepthToRGB<float>(depth_msg, registered_msg, depth_to_rgb);
                CreatePointCloud<float>(depth_msg, cloud_msg, rgb_model_, bb->bounding_boxes[i], 0.0);
            }
            else
            {
                ROS_INFO("Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
                return;
            }
            Clustering::PersonCloud(*cloud_msg);
            visualization_msgs::Marker marker_ = Clustering::GetPersonBoundingBoxes(cloud_msg, i);
            marker_array.markers.push_back(marker_);
        }
    }
    std::cout<<"In time: "<< start.toSec()<<"Out time: "<<ros::Time::now().toSec()<<"Time difference "<<ros::Time::now().toSec() - start.toSec()  <<"\n";
    Clustering::PublishBoxesArray(marker_array);
  
}




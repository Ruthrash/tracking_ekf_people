#include "tracking_ekf_people/PersonDetection.h"

PersonDetection::PersonDetection(){}
PersonDetection::~PersonDetection(){}

PersonDetection::PersonDetection(ros::NodeHandle &node) : Clustering(node), DepthImageProc(node)
{
    yolo_sync_sub.subscribe(node, "/darknet_ros/bounding_boxes",300);
    depth_sync_sub.subscribe(node, "/pepper_robot/camera/depth/image_raw",300);
    //depth_sync_sub.subscribe(node, "/camera/depth_registered/image",300);
    sync_.reset(new Sync(MySyncPolicy(3000), yolo_sync_sub, depth_sync_sub));
    sync_->registerCallback(boost::bind(&PersonDetection::SyncYOLODepthCB, this, _1, _2));
    //depth_info_sub = node.subscribe("/camera/depth_registered/camera_info", 1, &PersonDetection::DepthInfoCB, this);
    tf_buffer_.reset( new tf2_ros::Buffer );
    tf_.reset( new tf2_ros::TransformListener(*tf_buffer_) );
    depth_reg_pub = node.advertise<sensor_msgs::Image>("image_rect", 1000);
}




void PersonDetection::SyncYOLODepthCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& depth_msg)
{
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
                //std::cout<<"rgb_info frame id "<<rgb_info_msg->header.frame_id<<" depth_info frame id"<<depth_info_msg->header.frame_id<<std::endl; 
                std::string rgb_info_id = rgb_info.header.frame_id;
                //rgb_info_msg->header.frame_id = rgb_info.erase(0,1);
                std::string depth_info_id =     depth_info.header.frame_id;
                //depth_info_msg->header.frame_id = depth_info.erase(0,1);
                //depth_info = depth_info.erase(0,1);
                //rgb_info = rgb_info.erase(0,1);
                //rgb_info_msg->header.frame_id = rgb_info;
                //depth_info_msg->header.frame_id = depth_info;
                geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform (
                                    rgb_info_id, depth_info_id,
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
            //std::string rgb_info_id = rgb_info->header.frame_id;
            //std::cout<<"ulalalaaa "<<rgb_info<<std::endl;
            //rgb_info = rgb_info.erase(0,1);
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
                RegisterDepthToRGB<uint16_t>(depth_msg, registered_msg, depth_to_rgb);
                depth_reg_pub.publish(registered_msg);
                CreatePointCloud<uint16_t>(registered_msg, cloud_msg, depth_model_, bb->bounding_boxes[i], 0.0);
                
            }
            else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            {
                RegisterDepthToRGB<float>(depth_msg, registered_msg, depth_to_rgb);
                depth_reg_pub.publish(registered_msg);
                CreatePointCloud<float>(registered_msg, cloud_msg, depth_model_, bb->bounding_boxes[i], 0.0);
            }
            else
            {
                ROS_INFO("Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
                return;
            }
            visualization_msgs::Marker marker_ = Clustering::GetPersonBoundingBoxes(cloud_msg, i);
            //for(int j = 0; j < marker_array_d.markers.size() ; j++)
             //   marker_array.markers.push_back(marker_array_d.markers[j]);
            Clustering::PersonCloud(*cloud_msg);
            //ros::Duration(0.5).sleep();
            marker_array.markers.push_back(marker_);
           // marker_array.header = marker_.header;
        }
    }
    Clustering::PublishBoxesArray(marker_array);
    std::cout<<marker_array.markers.size()<<"vountttt\n";
    for(int idx = 0; idx <marker_array.markers.size(); idx++)
    {
        std::cout<<"box"<<marker_array.markers[idx].pose.position.x <<","<<marker_array.markers[idx].pose.position.y <<","<<marker_array.markers[idx].pose.position.z <<"\n";
        std::cout<<"scale"<<marker_array.markers[idx].scale.x<<","<<marker_array.markers[idx].scale.y<<","<<marker_array.markers[idx].scale.z<<"\n"; 
    }
  
}




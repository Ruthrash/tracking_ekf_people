#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

class Clustering
{
public:
    Clustering();
    ~Clustering();
    Clustering(ros::NodeHandle &node);

protected:
    void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions);
    visualization_msgs::Marker GetPersonBoundingBoxes(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void PublishBoxesArray(const visualization_msgs::MarkerArray &boxes);

private:

    ros::NodeHandle nh;
    ros::Publisher bbox_viz_pub; 



};

#endif
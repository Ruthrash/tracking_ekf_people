#include "tracking_ekf_people/Clustering.h"

Clustering::Clustering(){}
Clustering::~Clustering(){}

Clustering::Clustering(ros::NodeHandle &node)  
{
    nh = node;
    bbox_viz_pub = node.advertise<visualization_msgs::MarkerArray> ("/person_bounding_boxes",1,true);
}

void Clustering::PublishBoxesArray(const visualization_msgs::MarkerArray &boxes)
{
    bbox_viz_pub.publish(boxes);
}


void Clustering::GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions) 
{
    Eigen::Vector4f min_pt, max_pt;
    //pcl::PointXYZ pcl_min_pt, pcl_max_pt;
    //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr const_cloud = cloud;
    pcl::getMinMax3D( *cloud, min_pt, max_pt);

    //min_pt = pcl_min_pt.getRGB

    pose->position.x = (max_pt.x() + min_pt.x()) / 2;
    pose->position.y = (max_pt.y() + min_pt.y()) / 2;
    pose->position.z = (max_pt.z() + min_pt.z()) / 2;
    pose->orientation.w = 1;

    dimensions->x = max_pt.x() - min_pt.x();
    dimensions->y = max_pt.y() - min_pt.y();
    dimensions->z = max_pt.z() - min_pt.z();
}


visualization_msgs::Marker Clustering::GetPersonBoundingBoxes(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    std::cout<<"Trying clustering\n";
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform voxel grid downsampling filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr); sor.setLeafSize (0.01, 0.01, 0.01); sor.filter (*cloudFilteredPtr);//cloudFilteredPtr has voxelised pointcloud

    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

    // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

    // perform euclidean cluster segmentation to seporate individual objects
    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (xyzCloudPtr);

    // create the extraction object for the clusters
    std::vector<pcl::PointIndices> cluster_indices; pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // specify euclidean cluster parameters
    float cluster_tolerance = 0.03 , min_cluster_size = 500, max_cluster_size = 6500;
    nh.getParam("cluster_tolerance",cluster_tolerance);
    nh.getParam("min_cluster_size",min_cluster_size);
    nh.getParam("max_cluster_size",max_cluster_size);
    
    // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
    ec.setClusterTolerance (cluster_tolerance); ec.setMinClusterSize (min_cluster_size); ec.setMaxClusterSize (max_cluster_size);
    std::cout<<min_cluster_size<<", "<<max_cluster_size<<", "<<cluster_tolerance<<std::endl;
    ec.setSearchMethod (tree);
    ec.setInputCloud (xyzCloudPtr);
    ec.extract (cluster_indices);

    // declare the output variable instances
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 outputPCL;
    int count = 0;
    float max;//to get cluster with maximum size
    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::Marker max_object_marker;//to get object marker of maximum size cluster
    sensor_msgs::PointCloud2 max_cloud_msg; 
    
    std::cout<<cluster_indices.size()<<"fasdf\n";
    // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        // create a new clusterData message object //obj_recognition::ClusterData clusterData;
        // create a pcl object to hold the extracted cluster
        pcl::PointCloud<pcl::PointXYZ> *cluster = new pcl::PointCloud<pcl::PointXYZ>;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr (cluster);

        // now we are in a vector of indices pertaining to a single cluster.
        // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            clusterPtr->points.push_back(xyzCloudPtr->points[*pit]);
        }
        visualization_msgs::Marker object_marker;
        object_marker.ns = "objects";
        object_marker.id = ++count;
        object_marker.header.frame_id = cloud_msg->header.frame_id;
        object_marker.type = visualization_msgs::Marker::CUBE;   
        object_marker.color.g = 1;
        object_marker.color.a = 0.3;
        GetAxisAlignedBoundingBox(clusterPtr, &object_marker.pose, &object_marker.scale);//gets a bounding box for all clusters of PCL within YOLO boudning box
        float size = abs(object_marker.scale.x+object_marker.scale.y+object_marker.scale.z);
        pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);
        // Convert to ROS data type
        pcl_conversions::fromPCL(outputPCL, output);
        if(it==cluster_indices.begin())
        {
            max = abs(object_marker.scale.x+object_marker.scale.y+object_marker.scale.z);
            max_object_marker = object_marker;
            max_cloud_msg = output;
        }
        else if(size > max )//&& abs(object_marker.pose.position.y) <= 0.5)
        {
            max_object_marker = object_marker;
            max = abs(object_marker.scale.x+object_marker.scale.y+object_marker.scale.z); 
            max_cloud_msg = output;
            marker_array_.markers.push_back(object_marker);
        }
        
        std::cout<<"size= "<<max<<"\n";
        std::cout<<"frame_id= "<<cloud_msg->header.frame_id<<"\n";

    }
    PublishBoxesArray(marker_array_);
    return max_object_marker;

}



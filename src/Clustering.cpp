#include "tracking_ekf_people/Clustering.h"

Clustering::Clustering(){}
Clustering::~Clustering(){}

Clustering::Clustering(ros::NodeHandle &node)  
{
    nh = node;
    bbox_viz_pub = node.advertise<visualization_msgs::MarkerArray> ("/person_bounding_boxes",1,true);
    person_pcl_pub = node.advertise<sensor_msgs::PointCloud2> ("/person_cloud",1,true);
}

void Clustering::PublishBoxesArray(const visualization_msgs::MarkerArray &boxes)
{
    bbox_viz_pub.publish(boxes);
}
void Clustering::PersonCloud(const sensor_msgs::PointCloud2 &cloud_msg)
{
    person_pcl_pub.publish(cloud_msg);
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


visualization_msgs::Marker Clustering::GetPersonBoundingBoxes(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const int &id)
{
    //std::cout<<"Trying clustering\n";
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform voxel grid downsampling filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr); sor.setLeafSize (0.03, 0.03, 0.03); sor.filter (*cloudFilteredPtr);//cloudFilteredPtr has voxelised pointcloud



  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr_ (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr_);


  //perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (xyzCloudPtr_);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-2.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);



  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);


  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.04);

  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);

















    //remove ground plane 



    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_ = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr (xyz_cloud_); // need a boost shared pointer for pcl function inputs

    // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
    //pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


    visualization_msgs::Marker max_object_marker;
    max_object_marker.ns = "objects";
    max_object_marker.id = id;
    max_object_marker.header.frame_id = cloud_msg->header.frame_id;
    max_object_marker.type = visualization_msgs::Marker::CUBE;   
    max_object_marker.color.g = 1;
    max_object_marker.color.a = 0.3;


    //pcl::fromPCLPointCloud2(*xyzCloudPtrRansacFiltered, *xyzCloudPtr);
    xyzCloudPtr = xyzCloudPtrRansacFiltered;
    //GetAxisAlignedBoundingBox(xyzCloudPtr, &max_object_marker.pose, &max_object_marker.scale);
    // perform euclidean cluster segmentation to seporate individual objects
    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (xyzCloudPtr);

    // create the extraction object for the clusters
    std::vector<pcl::PointIndices> cluster_indices; pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // specify euclidean cluster parameters
    float cluster_tolerance, min_cluster_size, max_cluster_size;
    nh.getParam("cluster_tolerance",cluster_tolerance);
    nh.getParam("min_cluster_size",min_cluster_size);
    nh.getParam("max_cluster_size",max_cluster_size);
    
    // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
    ec.setClusterTolerance (cluster_tolerance); ec.setMinClusterSize (min_cluster_size); ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (xyzCloudPtr);
    ec.extract (cluster_indices);

    // declare the output variable instances
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 outputPCL;
    int count = 0;
    float max;//to get cluster with maximum size
    visualization_msgs::MarkerArray marker_array_;
    //sensor_msgs::PointCloud2 max_cloud_msg; 
    
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
        object_marker.id = id;
        object_marker.header.frame_id = cloud_msg->header.frame_id;
        object_marker.type = visualization_msgs::Marker::CUBE;   
        object_marker.color.g = 1;
        object_marker.color.a = 0.3;
        GetAxisAlignedBoundingBox(clusterPtr, &object_marker.pose, &object_marker.scale);//gets a bounding box for all clusters of PCL within YOLO boudning box
        float size = abs(object_marker.scale.x+object_marker.scale.y+object_marker.scale.z);
        //pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);
        // Convert to ROS data type
        //pcl_conversions::fromPCL(outputPCL, output);
        //marker_array_.markers.push_back(object_marker);
        if(it==cluster_indices.begin())
        {
            max = abs(object_marker.scale.x+object_marker.scale.y+object_marker.scale.z);
            max_object_marker = object_marker;
            //max_cloud_msg = output;
        }
        else if(size > max)
        {
            max_object_marker = object_marker;
            max = abs(object_marker.scale.x+object_marker.scale.y+object_marker.scale.z); 
            //max_cloud_msg = output;
        }
        //marker_array_.markers.push_back(object_marker);

    }
    std::cout<<"Max obj "<<max_object_marker.header.frame_id; 
    max_object_marker.header.stamp = ros::Time::now();
    ros::Duration d(1);
    max_object_marker.lifetime =d; 
    //
    //PublishBoxesArray(marker_array_);
    return max_object_marker;

}



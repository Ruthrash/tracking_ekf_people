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

private:

};



#endif 
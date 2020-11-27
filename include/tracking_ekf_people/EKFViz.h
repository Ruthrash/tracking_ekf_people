#ifndef EKF_VIZ_H
#define EKF_VIZ_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>

class EKFViz
{
public:
    ~EKFViz();
    EKFViz();
    EKFViz(ros::NodeHandle &node);

protected:

private:
    visualization_msgs::Marker person_marker;
    ros::Publisher bbox_viz_pub; 
    ros::Publisher velocity_viz_pub;
};


#endif
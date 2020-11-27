#include "tracking_3d_people/EKFViz.h"
EKFViz::~EKFViz(){}
EKFViz::EKFViz(){}

EKFViz::EKFViz(ros::NodeHandle &node)
{
    bbox_viz_pub = node.advertise<visualization_msgs::Marker> ("/person_bounding_box3d",1,true);
    velocity_viz_pub = node.advertise<geometry_msgs::TwistStamped> ("/person_velocity",1,true);
    person_marker.color.g = 1;
    person_marker.color.a = 0.3;
    person_marker.ns = "person"; 
    person_marker.type = visualization_msgs::Marker::CUBE;
}

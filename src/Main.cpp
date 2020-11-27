#include "tracking_ekf_people/PersonDetection.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle node;
    PersonDetection pd(node);
    ros::Rate loop_rate(10);
    while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0; 
}
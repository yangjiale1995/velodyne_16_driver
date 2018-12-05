#include <ros/ros.h>
#include <iostream>
#include "velodyne_laserscan/VelodyneLaserScan.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_laserscan_node");
	ros::NodeHandle nh;
	std::cout << "node.cpp" << std::endl;
	
	velodyne_laserscan::VelodyneLaserScan n(nh);
	ros::spin();
	return 0;
}

#include "table_recon/ObjectCloudNode.h"
#include <ros/ros.h>
#include <iostream>
using namespace std;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "table_recon");
	ObjectCloudNode objectCloudNode;
	ros::spin();
	return 0;
}

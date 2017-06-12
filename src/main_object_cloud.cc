#include "table_recon/System.h"
#include <ros/ros.h>
#include <iostream>
using namespace std;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "table_recon");
	System objectCloudNode;
	ros::spin();
	return 0;
}

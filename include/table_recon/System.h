#include "table_recon/Helper.h"

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <darknet_ros/DetectedObjects.h>

#include <ros/ros.h>

using namespace std;

class System
{
private:
	
	ros::NodeHandle nh;

	//Subscribers
	ros::Subscriber visualOdom_sub,
					pointCloud_sub,
					cameraInfo_sub,
					detectionWindow_sub,
					depthImg_sub,
					rgbImg_sub,
					mapData_sub;

	// Publishers
	ros::Publisher objectDepth_pub, objectImg_pub, objectCloud_pub;

	//Mutex locks	
	static pthread_mutex_t visualOdom_mutex;
	static pthread_mutex_t pointCloud_mutex;
	static pthread_mutex_t cameraInfo_mutex;
	static pthread_mutex_t depthImg_mutex;
	static pthread_mutex_t rgbImg_mutex;
	static pthread_mutex_t objects_mutex;
	
	nav_msgs::OdometryPtr visualOdomPtr;
	sensor_msgs::CameraInfo cameraInfo;
	sensor_msgs::Image depthImg, rgbImg;
	sensor_msgs::PointCloud2 pointCloud;
	
	ros::ServiceClient imageDetectionClient;
			
	//Callback Funtions
	void visualOdomCb(const nav_msgs::OdometryPtr odomPtr);
	void pointCloudCb(const sensor_msgs::PointCloud2Ptr pointCloudPtr);	
	void cameraInfoCb(const sensor_msgs::CameraInfoPtr cameraInfoPtr);	
	void depthImgCb(const sensor_msgs::ImagePtr depthImgPtr);	
	void rgbImgCb(const sensor_msgs::ImagePtr rgbImgPtr);	
	void detectionWindowCb(const darknet_ros::DetectedObjectsPtr objectsPtr);

protected:
	static const string WORLD_FRAME;

public:
	System();
	~System();
};
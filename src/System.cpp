#include <iostream>

#include "table_recon/System.h"

#include <darknet_ros/ObjectInfo.h>
#include <darknet_ros/ImageDetection.h>

using namespace std;

pthread_mutex_t System::visualOdom_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t System::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t System::cameraInfo_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t System::depthImg_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t System::rgbImg_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t System::objects_mutex = PTHREAD_MUTEX_INITIALIZER;
const string System::WORLD_FRAME = string("/map");

System::System()
{
	objectCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/object_pc",1);
	objectDepth_pub = nh.advertise<sensor_msgs::Image>("/object_depth",1);
	objectImg_pub = nh.advertise<sensor_msgs::Image>("/object_img",1);
	
	visualOdomPtr = nav_msgs::OdometryPtr();

	imageDetectionClient = nh.serviceClient<darknet_ros::ImageDetection>("/darknet_ros/detect_objects");

	visualOdom_sub = nh.subscribe("/rtabmap/odom",100, &System::visualOdomCb, this);
	pointCloud_sub = nh.subscribe("/camera/depth_registered/points", 100, &System::pointCloudCb, this);
	depthImg_sub = nh.subscribe("/camera/depth_registered/image_raw", 100, &System::depthImgCb, this);
	rgbImg_sub = nh.subscribe("/camera/rgb/image_rect_color", 100, &System::rgbImgCb, this);
	cameraInfo_sub = nh.subscribe("/camera/rgb/camera_info", 100, &System::cameraInfoCb, this);
	detectionWindow_sub = nh.subscribe("/darknet_ros/detected_objects", 100, &System::detectionWindowCb, this);
}

System::~System() {}

/**
 *	Receive Visual Odometry from rtabmap
 */
void System::visualOdomCb(const nav_msgs::OdometryPtr odomPtr)
{
	pthread_mutex_lock(&visualOdom_mutex);
	visualOdomPtr = odomPtr;
	pthread_mutex_unlock(&visualOdom_mutex);
}

/**
 *	Receive Depth Image
 */
void System::depthImgCb(const sensor_msgs::ImagePtr depthImgPtr)
{
	pthread_mutex_lock(&depthImg_mutex);
	depthImg = *depthImgPtr;
	pthread_mutex_unlock(&depthImg_mutex);
}

/**
 *	Receive RGB Image
 */
void System::rgbImgCb(const sensor_msgs::ImagePtr rgbImgPtr)
{
	pthread_mutex_lock(&rgbImg_mutex);
	rgbImg = *rgbImgPtr;
	pthread_mutex_unlock(&rgbImg_mutex);
}

/**
 *		Receive current visible dense point cloud
 */
void System::pointCloudCb(const sensor_msgs::PointCloud2Ptr pointCloudPtr)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	pointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

/**
 *	Receive Camera Info
 */
void System::cameraInfoCb(const sensor_msgs::CameraInfoPtr cameraInfoPtr)
{
	pthread_mutex_lock(&cameraInfo_mutex);
	cameraInfo = *cameraInfoPtr;
	pthread_mutex_unlock(&cameraInfo_mutex);
}	

/**
 *	Receive Detected Objects from YOLO
 */
void System::detectionWindowCb(const darknet_ros::DetectedObjectsPtr objectsPtr)
{
	pthread_mutex_lock(&objects_mutex);
	PcII result = Helper::getDetectedObjects(*objectsPtr, depthImg, rgbImg, cameraInfo);
	objectCloud_pub.publish(get<0>(result));
	objectDepth_pub.publish(get<1>(result));
	objectImg_pub.publish(get<2>(result));
	pthread_mutex_unlock(&objects_mutex);
}


#include <vector>
#include <utility>
#include <map>
#include <string>
//#include <tuple>

#include <tf/transform_datatypes.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32MultiArray.h>
#include <darknet_ros/DetectedObjects.h>
#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/NodeData.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>


using namespace std;

//typedef tuple<pcl::PointCloud<pcl::PointXYZRGB>, sensor_msgs::Image, sensor_msgs::Image> PcII;

class ObjectCloudNode
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
	ros::Publisher  objectWorldCloud_pub, objectCloud_pub, objectDepth_pub, objectImg_pub, objectMapData_pub;

	//Mutex locks	
	static pthread_mutex_t visualOdom_mutex;
	static pthread_mutex_t pointCloud_mutex;
	static pthread_mutex_t cameraInfo_mutex;
	static pthread_mutex_t depthImg_mutex;
	static pthread_mutex_t rgbImg_mutex;
	static pthread_mutex_t objects_mutex;
	static pthread_mutex_t mapData_mutex;
	
	nav_msgs::OdometryPtr visualOdomPtr;
	pcl::PointCloud<pcl::PointXYZRGB> pixelCloud, voxelCloud, objectCloud;
	pcl::PCLPointCloud2 objectCloud2;
	pcl::PointXYZRGB minPoint, maxPoint;
	sensor_msgs::CameraInfo cameraInfo;
	sensor_msgs::Image depthImg, rgbImg;
	sensor_msgs::PointCloud2 worldCloud, emptyCloud, currCloud;
	//cv_bridge::CvImagePtr depthPtr_, rgbPtr_;
	//tf::StampedTransform worldTF;
 	//sensor_msgs::PointCloud2 pc;
 	//float rc00,rc01,rc02,rc03,rc10,rc11,rc12,rc13,rc20,rc21,rc22,rc23; //transform matrix values
	vector<vector<float> > table_pos;
	int num_nodes;
	ros::ServiceClient imageDetectionClient;
	map<long unsigned int, pair<vector<unsigned char>, vector<unsigned char> > > kfObjectImages;
	map<long unsigned int, sensor_msgs::PointCloud2> kfObjectPoints;
	map<long unsigned int, sensor_msgs::PointCloud2>::iterator it;
	vector<long unsigned int> prevKfIDs, currKfIDs;

	//tf::TransformListener tfListener;
		
	//Callback Funtions
	void visualOdomCb(const nav_msgs::OdometryPtr odomPtr);
	void pointCloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);	
	void cameraInfoCb(const sensor_msgs::CameraInfoPtr cameraInfoPtr);	
	void depthImgCb(const sensor_msgs::ImagePtr depthImgPtr);	
	void rgbImgCb(const sensor_msgs::ImagePtr rgbImgPtr);	
	void detectionWindowCb(const darknet_ros::DetectedObjectsPtr objectsPtr);
	void mapDataCb(rtabmap_ros::MapDataPtr mapDataPtr);
	sensor_msgs::PointCloud2::Ptr detectedImage(darknet_ros::DetectedObjects detections, sensor_msgs::Image depth, sensor_msgs::Image rgb);
	sensor_msgs::PointCloud2 detectedImage(darknet_ros::DetectedObjects detections, sensor_msgs::Image depth, sensor_msgs::Image rgb, geometry_msgs::Pose pose);

	//pair<vector<unsigned char>, vector<unsigned char> > detectedImage(darknet_ros::DetectedObjects detections, sensor_msgs::Image depth, sensor_msgs::Image rgb);
	template<typename T>
	void convert(sensor_msgs::ImagePtr depth_msg,
                 sensor_msgs::ImagePtr rgb_msg,
				 sensor_msgs::PointCloud2::Ptr& cloud_msg);
	
	template<typename T>
	vector<T> set_diff(vector<T> v1, vector<T> v2);

protected:
	static const string WORLD_FRAME;

public:
	ObjectCloudNode();
	~ObjectCloudNode();
};
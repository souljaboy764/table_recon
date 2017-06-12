#include <tuple>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros/DetectedObjects.h>


using namespace std;

typedef tuple<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image> PcII;

class Helper
{
public:
	Helper();
	~Helper();
	template<typename T>
	static void convert(sensor_msgs::ImagePtr depth_msg,
                 sensor_msgs::ImagePtr rgb_msg,
                 sensor_msgs::CameraInfo cameraInfo,
				 sensor_msgs::PointCloud2::Ptr& cloud_msg);
	
	static PcII getDetectedObjects(darknet_ros::DetectedObjects detections, 
				 sensor_msgs::Image depth, 
				 sensor_msgs::Image rgb, 
				 sensor_msgs::CameraInfo cameraInfo);
};
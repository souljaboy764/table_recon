#include <iostream>
#include <string>
#include <algorithm>
#include <ctime>
#include <exception>

#include "table_recon/ObjectCloudNode.h"

#include <darknet_ros/ObjectInfo.h>
#include <darknet_ros/ImageDetection.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <rtabmap/core/Compression.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>
#include <depth_image_proc/depth_traits.h>
#include <pcl_ros/transforms.h>
#include <opencv/cv.h>


using namespace std;

pthread_mutex_t ObjectCloudNode::visualOdom_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::cameraInfo_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::depthImg_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::rgbImg_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::objects_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::mapData_mutex = PTHREAD_MUTEX_INITIALIZER;
const string ObjectCloudNode::WORLD_FRAME = string("/map");

ObjectCloudNode::ObjectCloudNode()
{
	srand (time(NULL));
	objectWorldCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/object_world",1);
	objectCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/object_pc",1);
	objectDepth_pub = nh.advertise<sensor_msgs::Image>("/object_depth",1);
	objectImg_pub = nh.advertise<sensor_msgs::Image>("/object_img",1);
	cout<<"init"<<endl;
	visualOdomPtr = nav_msgs::OdometryPtr();
	num_nodes = 0;
	objectCloud.width = 1;
	imageDetectionClient = nh.serviceClient<darknet_ros::ImageDetection>("/darknet_ros/detect_objects");
	
	emptyCloud.header.seq = 0;
	emptyCloud.header.stamp = ros::Time::now();
	emptyCloud.header.frame_id = WORLD_FRAME;
	emptyCloud.width = 1;
	emptyCloud.height = 0;
	emptyCloud.is_dense = false;
	emptyCloud.is_bigendian = false;
	sensor_msgs::PointCloud2Modifier pcd_modifier(emptyCloud);
	pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
	emptyCloud.data = vector<uint8_t>();

	worldCloud = sensor_msgs::PointCloud2(emptyCloud);
	worldCloud.header.seq = 0;

	/*objectCloud2.header.seq=0;
	objectCloud2.header.frame_id = "map";
	pcl_conversions::toPCL(ros::Time::now(), objectCloud2.header.stamp);*/


	cout<<"init done"<<endl;
	visualOdom_sub = nh.subscribe("/rtabmap/odom",100, &ObjectCloudNode::visualOdomCb, this);
	pointCloud_sub = nh.subscribe("/camera/depth_registered/points", 100, &ObjectCloudNode::pointCloudCb, this);
	depthImg_sub = nh.subscribe("/camera/depth_registered/image_raw", 100, &ObjectCloudNode::depthImgCb, this);
	rgbImg_sub = nh.subscribe("/camera/rgb/image_rect_color", 100, &ObjectCloudNode::rgbImgCb, this);
	cameraInfo_sub = nh.subscribe("/camera/rgb/camera_info", 100, &ObjectCloudNode::cameraInfoCb, this);
	detectionWindow_sub = nh.subscribe("/darknet_ros/detected_objects", 100, &ObjectCloudNode::detectionWindowCb, this);
	//mapData_sub = nh.subscribe("/rtabmap/mapData", 100, &ObjectCloudNode::mapDataCb, this);
}

ObjectCloudNode::~ObjectCloudNode()
{
	/*time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,80,"%Y%y%d_%I%M%S",timeinfo);
	
	pcl::io::savePCDFileASCII(string(buffer), objectCloud);*/
}

/**
 *	Function to convert depth image to PointCloud (Copied from depth_image_proc)
 */
template<typename T>
void ObjectCloudNode::convert(sensor_msgs::ImagePtr depth_msg,
							  sensor_msgs::ImagePtr rgb_msg,
							  sensor_msgs::PointCloud2::Ptr& cloud_msg)
{

	int red_offset, green_offset, blue_offset, color_step;
	if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8)
	{
		red_offset	 = 0;
		green_offset = 1;
		blue_offset	= 2;
		color_step	 = 3;
	}
	else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8)
	{
		red_offset	 = 2;
		green_offset = 1;
		blue_offset	= 0;
		color_step	 = 3;
	}
	else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8)
	{
		red_offset	 = 0;
		green_offset = 0;
		blue_offset	= 0;
		color_step	 = 1;
	}
	else
	{
		try
		{
			rgb_msg = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8)->toImageMsg();
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_INFO("Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
			return;
		}
		red_offset	 = 0;
		green_offset = 1;
		blue_offset	= 2;
		color_step	 = 3;
	}
	// Use correct principal point from calibration
	float center_x = cameraInfo.K[2];
	float center_y = cameraInfo.K[5];
	
	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters( T(1) );
	float constant_x = unit_scaling / cameraInfo.K[0];
	float constant_y = unit_scaling / cameraInfo.K[4];
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	
	const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
	int row_step = depth_msg->step / sizeof(T);
	const uint8_t* rgb = &rgb_msg->data[0];
	int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

	sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");

	for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip)
		for (int u = 0; u < int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
		{
			T depth = depth_row[u];


			// Check for invalid measurements
			if (!depth_image_proc::DepthTraits<T>::valid(depth))
			{
			*iter_x = *iter_y = *iter_z = bad_point;
			}
			else
			{
			// Fill in XYZ
			*iter_x = (u - center_x) * depth * constant_x;
			*iter_y = (v - center_y) * depth * constant_y;
			*iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);
			}

			// Fill in color
			*iter_r = rgb[red_offset];
			*iter_g = rgb[green_offset];
			*iter_b = rgb[blue_offset];
		}
}
template<typename T>
vector<T> ObjectCloudNode::set_diff(vector<T> v1, vector<T> v2)
{
	vector<T> v(max(v1.size(),v2.size()));					  
	typename vector<T>::iterator it = std::set_difference (v1.begin(), v1.end(), v2.begin(), v2.end(), v.begin());
	v.resize(it-v.begin());
	return v;
}

/**
 *	Receive Map Data from RTABMAP
 */
void ObjectCloudNode::mapDataCb(const rtabmap_ros::MapDataPtr mapDataPtr)
{
	pthread_mutex_lock(&mapData_mutex);
	currKfIDs.clear();
	cout<<"Curr Nodes: ";
	for(int i=0;i<mapDataPtr->nodes.size();i++)
	{
		cout<<mapDataPtr->nodes[i].id<<" ";
		currKfIDs.push_back(mapDataPtr->nodes[i].id);
	}
	cout<<endl;

	worldCloud.data.clear();
	worldCloud.data = vector<uint8_t>();
	worldCloud.height = 0;
	/*vector<long unsigned int> kfsToRemove = set_diff<long unsigned int>(prevKfIDs, currKfIDs);
	for(int i=0;i<kfsToRemove.size();i++)
		kfObjectPoints.erase(kfsToRemove[i]);*/
	vector<long unsigned int> kfsToAdd = set_diff<long unsigned int>(currKfIDs, prevKfIDs);
	prevKfIDs = vector<long unsigned int>(currKfIDs);
	//objectCloud.points.clear();
	for(int i=mapDataPtr->nodes.size() - 1;i>0;i--)
	{
		cout<<"finding "<<mapDataPtr->nodes[i].id<<endl;
		it = kfObjectPoints.find(mapDataPtr->nodes[i].id);
		if(it == kfObjectPoints.end())
		{
			cout<<mapDataPtr->nodes[i].id<<" not found"<<endl;
			darknet_ros::ImageDetection obj;
			cv_bridge::CvImagePtr rgbPtr = boost::make_shared<cv_bridge::CvImage>();
			cv_bridge::CvImagePtr depthPtr = boost::make_shared<cv_bridge::CvImage>();
			cout<<"uncompressing rgb"<<endl;
			rgbPtr->image = rtabmap::uncompressImage(mapDataPtr->nodes[i].image);
			if(rgbPtr->image.empty())
			{	
				cout<<"EMPTY RGB IMAGE"<<endl;
				continue;
			}
			rgbPtr->encoding = sensor_msgs::image_encodings::BGR8;
			cout<<"uncompressing depth"<<endl;
			depthPtr->image = rtabmap::uncompressImage(mapDataPtr->nodes[i].depth);
			if(depthPtr->image.empty())
			{
				cout<<"EMPTY DEPTH IMAGE"<<endl;
				continue;
			}
			depthPtr->encoding = depthPtr->image.empty()?"":depthPtr->image.type() == CV_32FC1?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
			cout<<"requesting srv"<<endl;
			obj.request.msg = *(rgbPtr->toImageMsg());
			imageDetectionClient.call(obj);
			cout<<"got objects"<<endl;
			/*sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2());
			cloud_msg->header = depthPtr->toImageMsg()->header;
			cloud_msg->header.frame_id = "camera_depth_optical_frame";
			cloud_msg->height = depthPtr->toImageMsg()->height;
			cloud_msg->width	= depthPtr->toImageMsg()->width;
			cloud_msg->is_dense = false;
			cloud_msg->is_bigendian = false;

			sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
			pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
			cout<<"made cloud_msg"<<endl;
			if (depthPtr->toImageMsg()->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
				convert<uint16_t>(depthPtr->toImageMsg(),rgbPtr->toImageMsg(), cloud_msg);
			else if (depthPtr->toImageMsg()->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
				convert<float>(depthPtr->toImageMsg(),rgbPtr->toImageMsg(), cloud_msg);
			cout<<"converted images to PCD"<<endl;*/
			/*pcl::PointCloud<pcl::PointXYZRGB> pc;
			pcl::fromROSMsg<pcl::PointXYZRGB>(*cloud_msg, pc);
			cout<<pc.height<<" "<<pc.width<<" "<<pc.points.size()<<endl;*/
			//sensor_msgs::PointCloud2 result = this->detectedImage(mapDataPtr->nodes.back().pose, obj.response.objects, *cloud_msg);
			geometry_msgs::Pose pose;
			memcpy(&(pose.position),&(mapDataPtr->nodes[i].localTransform[0].translation),3*sizeof(float));
			pose.orientation = mapDataPtr->nodes[i].localTransform[0].rotation;
			sensor_msgs::PointCloud2 result = detectedImage(obj.response.objects, *(depthPtr->toImageMsg()), *(rgbPtr->toImageMsg()), mapDataPtr->nodes[i].pose);
			cout<<"PCD added "<<mapDataPtr->nodes[i].id<<endl;
			kfObjectPoints[mapDataPtr->nodes[i].id] = result;
			cout<<result.height<<" "<<result.width<<endl;
			//kfObjectPoints[mapDataPtr->nodes[i].id] = result;
			//objectCloud.points.insert(objectCloud.points.end(), result.points.begin(), result.points.end());
			
		}
		if(!worldCloud.data.empty())
		{
			worldCloud.data.insert(worldCloud.data.end(), kfObjectPoints[mapDataPtr->nodes[i].id].data.begin(), kfObjectPoints[mapDataPtr->nodes[i].id].data.end());
			cout<<"ADDED TO END"<<endl;
		}
		else
		{
			worldCloud.data = kfObjectPoints[mapDataPtr->nodes[i].id].data;
			cout<<"ADDED TO EMPTY"<<endl;
		}
		worldCloud.height += kfObjectPoints[mapDataPtr->nodes[i].id].height * kfObjectPoints[mapDataPtr->nodes[i].id].width;
		worldCloud.row_step = worldCloud.point_step * worldCloud.height;
		cout<<"PCD added "<<kfObjectPoints[mapDataPtr->nodes[i].id].height * kfObjectPoints[mapDataPtr->nodes[i].id].width<<endl;
		//objectWorldCloud_pub.publish(kfObjectPoints[mapDataPtr->nodes[i].id]);
	}
	/*cout<<"PCD publishing"<<endl;
	for(int i=0; i<mapDataPtr->nodes.size();i++)
		*/
	/*objectCloud.height = objectCloud.points.size();
	pcl::toPCLPointCloud2(objectCloud, objectCloud2);*/
	/*pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(boost::make_shared<const pcl::PCLPointCloud2>(objectCloud2));
	sor.setLeafSize(0.05f, 0.05f, 0.05f);
	sor.filter(objectCloud2);
	pcl::fromPCLPointCloud2(objectCloud2, objectCloud);
	objectCloud.width = 1;
	objectCloud.height = objectCloud.points.size();*/
	/*sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(objectCloud, pc);*/
	cout<<"incrementing seq"<<endl;
	worldCloud.header.seq++;
	cout<<"setting stamp"<<endl;
	worldCloud.header.stamp = ros::Time::now();
	cout<<"PCD publishing"<<endl;
	objectWorldCloud_pub.publish(worldCloud);	
	cout<<"PCD published"<<endl;
	/*objectCloud2.header.seq++;
	objectCloud2.header.frame_id = WORLD_FRAME;
	pcl_conversions::toPCL(ros::Time::now(), objectCloud2.header.stamp);

	objectWorldCloud_pub.publish(objectCloud2);
	cout<<objectCloud2.height * objectCloud2.width<<endl;
	cout<<objectCloud.points.size()<<endl;*/
	pthread_mutex_unlock(&mapData_mutex);
}

/**
 *	Receive Visual Odometry from rtabmap
 */
void ObjectCloudNode::visualOdomCb(const nav_msgs::OdometryPtr odomPtr)
{
	pthread_mutex_lock(&visualOdom_mutex);
	visualOdomPtr = odomPtr;
	pthread_mutex_unlock(&visualOdom_mutex);
}

/**
 *	Receive Depth Image
 */
void ObjectCloudNode::depthImgCb(const sensor_msgs::ImagePtr depthImgPtr)
{
	pthread_mutex_lock(&depthImg_mutex);
	depthImg = *depthImgPtr;
	pthread_mutex_unlock(&depthImg_mutex);
}

/**
 *	Receive RGB Image
 */
void ObjectCloudNode::rgbImgCb(const sensor_msgs::ImagePtr rgbImgPtr)
{
	pthread_mutex_lock(&rgbImg_mutex);
	rgbImg = *rgbImgPtr;
	pthread_mutex_unlock(&rgbImg_mutex);
}

/**
 *		Receive current visible dense point cloud
 */
void ObjectCloudNode::pointCloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	pixelCloud = *cloud;
	pcl::toROSMsg(pixelCloud, currCloud);
	/*pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2 ());
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize (0.1f, 0.1f, 0.1f);
	sor.filter (*cloud_filtered);*/
	//pixelCloud.width = pixelCloud.points.size();
	//objectCloud_pub.publish(*cloud_filtered);
	pthread_mutex_unlock(&pointCloud_mutex);
}

/**
 *	Receive PTAM Info
 */
void ObjectCloudNode::cameraInfoCb(const sensor_msgs::CameraInfoPtr cameraInfoPtr)
{
	pthread_mutex_lock(&cameraInfo_mutex);
	cameraInfo = *cameraInfoPtr;
	pthread_mutex_unlock(&cameraInfo_mutex);
}	

void ObjectCloudNode::detectionWindowCb(const darknet_ros::DetectedObjectsPtr objectsPtr)
{
	pthread_mutex_lock(&objects_mutex);
	objectWorldCloud_pub.publish(detectedImage(*objectsPtr, depthImg, rgbImg, visualOdomPtr->pose.pose));
	pthread_mutex_unlock(&objects_mutex);
}

sensor_msgs::PointCloud2::Ptr ObjectCloudNode::detectedImage(darknet_ros::DetectedObjects detections, sensor_msgs::Image depth, sensor_msgs::Image rgb)
{
	try 
	{
		cv_bridge::CvImagePtr depthPtr_ = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1 );
		cv_bridge::CvImagePtr rgbPtr_ = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8 );

		cv::Mat depthTarget(depthPtr_->image.size(), depthPtr_->image.type()),
				rgbTarget(rgbPtr_->image.size(), rgbPtr_->image.type()); 

		depthTarget = cv::Scalar(0);
		rgbTarget = cv::Scalar(0,0,0);
		
		for(int i=0;i<detections.objects.size();i++)
		{
			if((!detections.objects[i].type.compare("chair") or !detections.objects[i].type.compare("table")) )
				//and detections.objects[i].prob>0.4)	
			{
				darknet_ros::ObjectInfo obj = detections.objects[i];
				//keeping bounding box within image dimensions
				if(obj.tl_x+obj.width >= rgb.width)
					obj.width = rgb.width-obj.tl_x;
				if(obj.tl_y+obj.height >= rgb.height)
					obj.height = rgb.height-obj.tl_y;

				cv::Mat subImage = depthTarget(cv::Rect(obj.tl_x, obj.tl_y, obj.width, obj.height));
				depthPtr_->image(cv::Rect(obj.tl_x, obj.tl_y, obj.width, obj.height)).copyTo(subImage);

				subImage = rgbTarget(cv::Rect(obj.tl_x, obj.tl_y, obj.width, obj.height));
				rgbPtr_->image(cv::Rect(obj.tl_x, obj.tl_y, obj.width, obj.height)).copyTo(subImage);
			}
		}
		sensor_msgs::PointCloud2::Ptr objPC(new sensor_msgs::PointCloud2());
		objPC->header = depthPtr_->toImageMsg()->header;
		objPC->height = depthPtr_->toImageMsg()->height;
		objPC->width	= depthPtr_->toImageMsg()->width;
		objPC->is_dense = false;
		objPC->is_bigendian = false;

		sensor_msgs::ImagePtr objDepthImg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthTarget).toImageMsg();
		objDepthImg->header.frame_id = "camera_rgb_optical_frame";
		objDepthImg->header.stamp = ros::Time::now();
		
		sensor_msgs::ImagePtr objRGBImg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbTarget).toImageMsg();
		objRGBImg->header.frame_id = "camera_rgb_optical_frame";
		objRGBImg->header.stamp = ros::Time::now();

		sensor_msgs::PointCloud2Modifier pcd_modifier(*objPC);
		pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
		
		if (depthPtr_->toImageMsg()->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
			convert<uint16_t>(objDepthImg, objRGBImg, objPC);
		else if (depthPtr_->toImageMsg()->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
			convert<float>(objDepthImg, objRGBImg, objPC);
		
		objPC->header.frame_id = "camera_rgb_optical_frame";
		objPC->header.stamp = ros::Time::now();

			/*sensor_msgs::ImagePtr msg_1 = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthTarget).toImageMsg();
			msg_1->header.frame_id = "camera_rgb_optical_frame";
			msg_1->header.stamp = ros::Time::now();
			objectDepth_pub.publish(msg_1);

			msg_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbTarget).toImageMsg();
			objectImg_pub.publish(msg_1);*/
		
		return objPC;
		
	} catch (exception &e)
	{
		cout<<e.what()<<endl;
		return sensor_msgs::PointCloud2Ptr();
	}
}

sensor_msgs::PointCloud2 ObjectCloudNode::detectedImage(darknet_ros::DetectedObjects detections, sensor_msgs::Image depth, sensor_msgs::Image rgb, geometry_msgs::Pose pose)
{
	cout<<"getting in camera frame"<<endl;
	sensor_msgs::PointCloud2::Ptr result = detectedImage(detections, depth, rgb);
	/*if(result->height*result->width==0)
		return *result;*/
	tf::StampedTransform worldTF;
	cout<<"getting TF"<<endl;
	tf::poseMsgToTF(pose, worldTF);	

	float rc00,rc01,rc02,rc03,rc10,rc11,rc12,rc13,rc20,rc21,rc22,rc23; //transform matrix values
	cout<<"getting tf coeffs"<<endl;
	rc00 = worldTF.getBasis()[0][0];  rc01 = worldTF.getBasis()[0][1];
	rc02 = worldTF.getBasis()[0][2];  rc03 = worldTF.getOrigin()[0];
	rc10 = worldTF.getBasis()[1][0];  rc11 = worldTF.getBasis()[1][1];
	rc12 = worldTF.getBasis()[1][2];  rc13 = worldTF.getOrigin()[1];
	rc20 = worldTF.getBasis()[2][0];  rc21 = worldTF.getBasis()[2][1];
	rc22 = worldTF.getBasis()[2][2];  rc23 = worldTF.getOrigin()[2];
	cout<<"initializing iterators"<<endl;
	sensor_msgs::PointCloud2Iterator<float> iter_x(*result, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*result, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*result, "z");
	cout<<"starting iterations"<<endl;
	for(int v = 0; v < int(result->height); ++v)
		for(int u = 0; u < int(result->width); ++u, ++iter_x, ++iter_y, ++iter_z)
		{
			float x = *iter_x;
			float y = *iter_y;
			float z = *iter_z;
		
			*iter_x = rc00*z - rc01*x - rc02*y + rc03;
			*iter_y = rc10*z - rc11*x - rc12*y + rc13;
			*iter_z = rc20*z - rc21*x - rc22*y + rc23;
		}
	cout<<"finished iterations"<<endl;
	result->header.frame_id = WORLD_FRAME;
	return *result;
}
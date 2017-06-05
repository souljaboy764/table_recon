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

#include <opencv/cv.h>


using namespace std;

pthread_mutex_t ObjectCloudNode::visualOdom_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::cameraInfo_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::depthImg_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ObjectCloudNode::rgbImg_mutex = PTHREAD_MUTEX_INITIALIZER;

ObjectCloudNode::ObjectCloudNode()
{
	srand (time(NULL));
	objectWorldCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/object_world",1);
	objectCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/object_pc",1);
	objectDepth_pub = nh.advertise<sensor_msgs::Image>("/object_depth",1);
	objectImg_pub = nh.advertise<sensor_msgs::Image>("/object_img",1);
	//cout<<"init"<<endl;
	visualOdomPtr = nav_msgs::OdometryPtr();
	num_nodes = 0;
	objectCloud.width = 1;
	imageDetectionClient = nh.serviceClient<darknet_ros::ImageDetection>("/darknet_ros/detect_objects");
	
	objectCloud2.header.seq=0;
	objectCloud2.header.frame_id = "map";
	pcl_conversions::toPCL(ros::Time::now(), objectCloud2.header.stamp);

	//cout<<"init done"<<endl;
	visualOdom_sub = nh.subscribe("/rtabmap/odom",100, &ObjectCloudNode::visualOdomCb, this);
	pointCloud_sub = nh.subscribe("/camera/depth_registered/points", 100, &ObjectCloudNode::pointCloudCb, this);
	depthImg_sub = nh.subscribe("/camera/depth_registered/image_raw", 100, &ObjectCloudNode::depthImgCb, this);
	rgbImg_sub = nh.subscribe("/camera/rgb/image_rect_color", 100, &ObjectCloudNode::rgbImgCb, this);
	cameraInfo_sub = nh.subscribe("/camera/rgb/camera_info", 100, &ObjectCloudNode::cameraInfoCb, this);
	detectionWindow_sub = nh.subscribe("/darknet_ros/detected_objects", 100, &ObjectCloudNode::detectionWindowCb, this);
	mapData_sub = nh.subscribe("/rtabmap/mapData", 100, &ObjectCloudNode::mapDataCb, this);
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
	cout<<"Offsets "<<red_offset<<" "<<blue_offset<<" "<<green_offset<<" "<<color_step<<endl;

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
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

	for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip)
		for (int u = 0; u < int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
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
			*iter_a = 255;
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
	currKfIDs.clear();
	for(int i=0;i<mapDataPtr->nodes.size();i++)
	{
		cout<<mapDataPtr->nodes[i].id<<" ";
		currKfIDs.push_back(mapDataPtr->nodes[i].id);
	}
	cout<<endl;
/*
	vector<long unsigned int> kfsToRemove = set_diff<long unsigned int>(prevKfIDs, currKfIDs);
	for(int i=0;i<kfsToRemove.size();i++)
		kfObjectPoints.erase(kfsToRemove[i]);
*/
	//objectCloud.points.clear();
	//for(int i=0;i<mapDataPtr->nodes.size();i++)
	{
		/*it = kfObjectPoints.find(mapDataPtr->nodes[i].id);
		if(it != kfObjectPoints.end())
			objectCloud.points.insert(objectCloud.points.end(), it->second.points.begin(), it->second.points.end());
		else*/
		{
			darknet_ros::ImageDetection obj;
			cv_bridge::CvImagePtr rgbPtr = boost::make_shared<cv_bridge::CvImage>();
			cv_bridge::CvImagePtr depthPtr = boost::make_shared<cv_bridge::CvImage>();
			rgbPtr->image = rtabmap::uncompressImage(mapDataPtr->nodes.back().image);
			cout<<rgbPtr->image.type()<<endl;
			rgbPtr->encoding = sensor_msgs::image_encodings::BGR8;
			depthPtr->image = rtabmap::uncompressImage(mapDataPtr->nodes.back().depth);
			depthPtr->encoding = depthPtr->image.empty()?"":depthPtr->image.type() == CV_32FC1?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
			obj.request.msg = *(rgbPtr->toImageMsg());
			imageDetectionClient.call(obj);
			
			sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2());
			cloud_msg->header = depthPtr->toImageMsg()->header;
			cloud_msg->header.frame_id = "camera_depth_optical_frame";
			cloud_msg->height = depthPtr->toImageMsg()->height;
			cloud_msg->width	= depthPtr->toImageMsg()->width;
			cloud_msg->is_dense = false;
			cloud_msg->is_bigendian = false;

			sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
			pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

			if (depthPtr->toImageMsg()->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
				convert<uint16_t>(depthPtr->toImageMsg(),rgbPtr->toImageMsg(), cloud_msg);
			else if (depthPtr->toImageMsg()->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
				convert<float>(depthPtr->toImageMsg(),rgbPtr->toImageMsg(), cloud_msg);

			pcl::PointCloud<pcl::PointXYZRGB> pc;
			pcl::fromROSMsg<pcl::PointXYZRGB>(*cloud_msg, pc);
			cout<<pc.height<<" "<<pc.width<<" "<<pc.points.size()<<endl;
			pcl::PointCloud<pcl::PointXYZRGB> result = this->detectedImage(mapDataPtr->nodes.back().pose, obj.response.objects, pc);
			cout<<result.height<<" "<<result.width<<" "<<result.points.size()<<endl;
			//kfObjectPoints[mapDataPtr->nodes[i].id] = result;
			objectCloud.points.insert(objectCloud.points.end(), result.points.begin(), result.points.end());
		}
	}
	objectCloud.height = objectCloud.points.size();
	pcl::toPCLPointCloud2(objectCloud, objectCloud2);
	/*pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(boost::make_shared<const pcl::PCLPointCloud2>(objectCloud2));
	sor.setLeafSize(0.05f, 0.05f, 0.05f);
	sor.filter(objectCloud2);
	pcl::fromPCLPointCloud2(objectCloud2, objectCloud);
	objectCloud.width = 1;
	objectCloud.height = objectCloud.points.size();*/
	/*sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(objectCloud, pc);*/
	
	objectCloud2.header.seq++;
	objectCloud2.header.frame_id = "map";
	pcl_conversions::toPCL(ros::Time::now(), objectCloud2.header.stamp);

	objectWorldCloud_pub.publish(objectCloud2);
	cout<<objectCloud2.height * objectCloud2.width<<endl;
	cout<<objectCloud.points.size()<<endl;
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
	
	/*pcl::PointCloud<pcl::PointXYZRGB> cloud = pcl::PointCloud<pcl::PointXYZRGB>(depthImg.width, depthImg.height, pcl::PointXYZRGB());
	cloud.header.frame_id = "map";
	cloud.height = depthImg.height;
	cloud.width = depthImg.width
	for(int i=0;i<depthImg.height;i++)
		for(int j=0;j<depthImg.width;j++)
		{
			pcl::PointXYZRGB p;
			p.z = 0;
			p.x = cameraInfo.K[2] + cameraInfo.K[0]*p.x/p.z;
			p.y = cameraInfo.K[5] + cameraInfo.K[4]*p.y/p.z;
			
			cloud.points.append(p);
		}*/
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
	//PcII result = this->detectedImage(rgbImg, depthImg , visualOdomPtr->pose.pose, *objectsPtr);
	//pcl::PointCloud<pcl::PointXYZRGB> cloud = this->detectedImage(visualOdomPtr->pose.pose, *objectsPtr, pixelCloud);
	//cout<<cloud.points.size()<<" Points found in current image"<<endl;
	/*sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(cloud, pc);
	pc.header.frame_id = "map";
	pc.header.stamp = ros::Time::now();
	objectCloud_pub.publish(pc);*/

	//sensor_msgs::PointCloud2 pc;
	/*pcl::toROSMsg(objectCloud, pc);
	pc.header.frame_id = "map";
	pc.header.stamp = ros::Time::now();
	objectWorldCloud_pub.publish(pc);*/
	objectWorldCloud_pub.publish(objectCloud2)
;	/*objectDepth_pub.publish(get<1>(result));
	objectImg_pub.publish(get<2>(result));		*/
}

pcl::PointCloud<pcl::PointXYZRGB> ObjectCloudNode::detectedImage(geometry_msgs::Pose pose, darknet_ros::DetectedObjects detections, pcl::PointCloud<pcl::PointXYZRGB> points)	
{
	try 
	{
		tf::StampedTransform worldTF;
		tf::poseMsgToTF(pose, worldTF);
		float rc00,rc01,rc02,rc03,rc10,rc11,rc12,rc13,rc20,rc21,rc22,rc23; //transform matrix values

		rc00 = worldTF.getBasis()[0][0];  rc01 = worldTF.getBasis()[0][1];
		rc02 = worldTF.getBasis()[0][2];  rc03 = worldTF.getOrigin()[0];
		rc10 = worldTF.getBasis()[1][0];  rc11 = worldTF.getBasis()[1][1];
		rc12 = worldTF.getBasis()[1][2];  rc13 = worldTF.getOrigin()[1];
		rc20 = worldTF.getBasis()[2][0];  rc21 = worldTF.getBasis()[2][1];
		rc22 = worldTF.getBasis()[2][2];  rc23 = worldTF.getOrigin()[2];
		
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		cloud.width = 1;

		float minDepth=255, maxDepth=0, meanDepth = 0;
		cout<<detections.objects.size()<<" Objects found"<<endl;
		for(int i=0;i<detections.objects.size();i++)
		{
			float xc = 0, yc = 0, zc = 0;
			if((!detections.objects[i].type.compare("chair") or !detections.objects[i].type.compare("table")) )
				//and detections.objects[i].prob>0.4)	
			{
				darknet_ros::ObjectInfo obj = detections.objects[i];
				//cout<<detections.objects[i].type<<" found"<<endl;
				//cout<<points.width<<" "<<points.height<<endl;
				//keeping bounding box within image dimensions
				/*if(obj.tl_x+obj.width >= minWidth)
					obj.width = minWidth-obj.tl_x;
				if(obj.tl_y+obj.height >= minHeight)
					obj.height = minHeight-obj.tl_y;*/
				//cout<<"cropped height and width"<<endl;
			
				//getting min and max depth in bounding box and calculating mean depth
				for(int j=obj.tl_x;j<obj.tl_x+obj.width;j+=2)
					for(int k=obj.tl_y;k<obj.tl_y+obj.height;k+=2)
					{
						if(points.points[points.width*k + j].z < minDepth)
							minDepth = points.points[points.width*k + j].z;
						if(points.points[points.width*k + j].z > maxDepth)
							maxDepth = points.points[points.width*k + j].z;
					}
			
				meanDepth = (minDepth + maxDepth)/2.0;
				//cout<<"got mean depth "<<meanDepth<<" "<<maxDepth<<endl;

				//storing only those points which are closer than mean depth
				for(int j=obj.tl_x;j<obj.tl_x+obj.width;j++)
					for(int k=obj.tl_y;k<obj.tl_y+obj.height;k++)
						if (points.points[points.width*k + j].z < meanDepth)
						{
							pcl::PointXYZRGB p, q = points.points[points.width*k + j];
							p.x = rc00*q.z - rc01*q.x - rc02*q.y + rc03;
							p.y = rc10*q.z - rc11*q.x - rc12*q.y + rc13;
							p.z = rc20*q.z - rc21*q.x - rc22*q.y + rc23;
							p.r = q.r;
							p.g = q.g;
							p.b = q.b;
							xc += p.x; yc += p.y; zc += p.z;
							cloud.points.push_back(p);
						}
				//cout<<"Stored "<<cloud.points.size()<<" points"<<endl;
				

			}
			/*if(cloud.points.size()==0)
				continue;
			xc /= cloud.points.size(); yc /= cloud.points.size(); zc /= cloud.points.size();
			
			//cout<<xc<<" "<<yc<<" "<<zc<<endl;
			if(xc==0 and yc==0 and zc==0)
				continue;
			else if(!table_pos.size())
			{
				table_pos.push_back(vector<uchar>());
				table_pos.back().push_back(xc);
				table_pos.back().push_back(yc);
				table_pos.back().push_back(zc);
			}
			else
			{
				bool new_table = true;
				float minDist = 1,dist;
				//cout<<"ELSE"<<endl;
				for(int i=0;i<table_pos.size();i++)
				{
					dist =  (table_pos[i][0]-xc)*(table_pos[i][0]-xc) + 
							(table_pos[i][1]-yc)*(table_pos[i][1]-yc) + 
							(table_pos[i][2]-zc)*(table_pos[i][2]-zc);
					//cout<< boolalpha <<dist<<" "<<(dist < minDist)<<endl;
					if(dist < minDist)
					{
						new_table = false;
						break;
					}
				}
				if(new_table)
				{
					table_pos.push_back(vector<uchar>());
					table_pos.back().push_back(xc);
					table_pos.back().push_back(yc);
					table_pos.back().push_back(zc);
				}
			}*/
		}
		
		cloud.height = cloud.points.size();
		return cloud;
		
	} catch (exception &e)
	{
		//cout<<e.what()<<endl;
		return pixelCloud;
	}

/*

	unsigned char* dat = &(objectCloud2.data[0]);
	unsigned int n=0;
	
	for(std::vector<MapPoint::Ptr>::iterator it=mpMap->vpPoints.begin(); it!=mpMap->vpPoints.end(); ++it,++n)
	{
	  if(n>objectCloud2.width-1) break;
	  MapPoint& p = *(*it);

	  Vector<3,float> fvec = p.v3WorldPos,pvec;
	  uint32_t colorlvl = 0xff<<((3-p.nSourceLevel)*8);
	  uint32_t lvl = p.nSourceLevel;
	  uint32_t KF = p.pPatchSourceKF->ID;

	  memcpy(dat, &(fvec),3*sizeof(float));
	  memcpy(dat+3*sizeof(uint32_t),&colorlvl,sizeof(uint32_t));
	  memcpy(dat+4*sizeof(uint32_t),&lvl,sizeof(uint32_t));
	  memcpy(dat+5*sizeof(uint32_t),&KF,sizeof(uint32_t));
	  dat+=objectCloud2.point_step;

	}*/
}

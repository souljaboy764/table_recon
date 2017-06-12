#include <iostream>
#include <algorithm>
#include <exception>

#include <stdio.h>

#include "table_recon/Helper.h"

#include <darknet_ros/ObjectInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <depth_image_proc/depth_conversions.h>
#include <depth_image_proc/depth_traits.h>
#include <opencv/cv.h>


using namespace std;

/**
 *	Function to convert depth image to PointCloud (Taken from depth_image_proc)
 */
template<typename T>
void Helper::convert(sensor_msgs::ImagePtr depth_msg,
							  sensor_msgs::ImagePtr rgb_msg,
							  sensor_msgs::CameraInfo cameraInfo,
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
			printf("Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
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

PcII Helper::getDetectedObjects(darknet_ros::DetectedObjects detections, sensor_msgs::Image depth, sensor_msgs::Image rgb, sensor_msgs::CameraInfo cameraInfo)
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
		objDepthImg->header.stamp = ros::Time::now();
		
		sensor_msgs::ImagePtr objRGBImg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbTarget).toImageMsg();
		objRGBImg->header.stamp = ros::Time::now();

		sensor_msgs::PointCloud2Modifier pcd_modifier(*objPC);
		pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
		
		if (depthPtr_->toImageMsg()->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
			convert<uint16_t>(objDepthImg, objRGBImg, cameraInfo, objPC);
		else if (depthPtr_->toImageMsg()->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
			convert<float>(objDepthImg, objRGBImg, cameraInfo, objPC);
		
		return make_tuple(*objPC, *objDepthImg, *objRGBImg);
		
	} catch (exception &e)
	{
		cout<<e.what()<<endl;
		return make_tuple(sensor_msgs::PointCloud2(), depth, rgb);
	}
}

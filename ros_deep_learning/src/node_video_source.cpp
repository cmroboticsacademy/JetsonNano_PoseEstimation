/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-utils/videoSource.h>
#include <sensor_msgs/CameraInfo.h>

// globals	
videoSource* stream = NULL;
imageConverter* image_cvt = NULL;
Publisher<sensor_msgs::Image> image_pub = NULL;
Publisher<sensor_msgs::CameraInfo> camera_info_pub = NULL;
std::string camera_info_dm = "";
std::vector<double> camera_info_d;
boost::array<double, 9> camera_info_k;
boost::array<double, 9> camera_info_r;
boost::array<double, 12> camera_info_p;
int camera_info_bx = 0;
int camera_info_by = 0;


// aquire and publish camera frame
bool aquireFrame()
{
	imageConverter::PixelType* nextFrame = NULL;

	// get the latest frame
	if( !stream->Capture(&nextFrame, 1000) )
	{
		ROS_ERROR("failed to capture next frame");
		return false;
	}

	// assure correct image size
	if( !image_cvt->Resize(stream->GetWidth(), stream->GetHeight(), imageConverter::ROSOutputFormat) )
	{
		ROS_ERROR("failed to resize camera image converter");
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;
	sensor_msgs::CameraInfo msg_info;

	if( !image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame) )
	{
		ROS_ERROR("failed to convert video stream frame to sensor_msgs::Image");
		return false;
	}

	msg_info.height = msg.height;
	msg_info.width = msg.width;
	msg_info.distortion_model = camera_info_dm;
	msg_info.D = camera_info_d;
	msg_info.K = camera_info_k;
	msg_info.R = camera_info_r;
	msg_info.P = camera_info_p;
	msg_info.binning_x = camera_info_bx;
	msg_info.binning_y = camera_info_by;

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();
	msg_info.header.stamp = msg.header.stamp;

	// publish the message
	image_pub->publish(msg);
	camera_info_pub->publish(msg_info);
	ROS_DEBUG("published %ux%u video frame", stream->GetWidth(), stream->GetHeight());
	
	return true;
}


// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	ROS_CREATE_NODE("video_source");

	/*
	 * declare parameters
	 */
	videoOptions video_options;

	std::string resource_str;
	std::string codec_str;
	std::string flip_str;
	
	int video_width = video_options.width;
	int video_height = video_options.height;
	int rtsp_latency = video_options.latency;

	std::vector<double> temp_k;
	std::vector<double> temp_r;
	std::vector<double> temp_p;
	
	ROS_DECLARE_PARAMETER("resource", resource_str);
	ROS_DECLARE_PARAMETER("codec", codec_str);
	ROS_DECLARE_PARAMETER("width", video_width);
	ROS_DECLARE_PARAMETER("height", video_height);
	ROS_DECLARE_PARAMETER("framerate", video_options.frameRate);
	ROS_DECLARE_PARAMETER("loop", video_options.loop);
	ROS_DECLARE_PARAMETER("flip", flip_str);
	ROS_DECLARE_PARAMETER("rtsp_latency", rtsp_latency);
	ROS_DECLARE_PARAMETER("camera_info_distortion_model", camera_info_dm);
	ROS_DECLARE_PARAMETER("camera_info_d", camera_info_d);
	ROS_DECLARE_PARAMETER("camera_info_k", temp_k);
	ROS_DECLARE_PARAMETER("camera_info_r", temp_r);
	ROS_DECLARE_PARAMETER("camera_info_p", temp_p);
	ROS_DECLARE_PARAMETER("camera_info_binning_x", camera_info_bx);
	ROS_DECLARE_PARAMETER("camera_info_binning_y", camera_info_by);
	/*
	 * retrieve parameters
	 */
	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("width", video_width);
	ROS_GET_PARAMETER("height", video_height);
	ROS_GET_PARAMETER("framerate", video_options.frameRate);
	ROS_GET_PARAMETER("loop", video_options.loop);
	ROS_GET_PARAMETER("flip", flip_str);
	ROS_GET_PARAMETER("rtsp_latency", rtsp_latency);
	ROS_GET_PARAMETER("camera_info_distortion_model", camera_info_dm);
	ROS_GET_PARAMETER("camera_info_d", camera_info_d);
	ROS_GET_PARAMETER("camera_info_k", temp_k);
	ROS_GET_PARAMETER("camera_info_r", temp_r);
	ROS_GET_PARAMETER("camera_info_p", temp_p);
	ROS_GET_PARAMETER("camera_info_binning_x", camera_info_bx);
	ROS_GET_PARAMETER("camera_info_binning_y", camera_info_by);

	std::copy(temp_k.begin(),temp_k.end(),camera_info_k.begin());
	std::copy(temp_r.begin(),temp_r.end(),camera_info_r.begin());
	std::copy(temp_p.begin(),temp_p.end(),camera_info_p.begin());
	
	if( resource_str.size() == 0 )
	{
		ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
		return 0;
	}

	if( codec_str.size() != 0 )
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

	if( flip_str.size() != 0 )
		video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());
	
	video_options.width = video_width;
	video_options.height = video_height;
	video_options.latency = rtsp_latency;
	
	ROS_INFO("opening video source: %s", resource_str.c_str());

	/*
	 * open video source
	 */
	stream = videoSource::Create(resource_str.c_str(), video_options);

	if( !stream )
	{
		ROS_ERROR("failed to open video source");
		return 0;
	}


	/*
	 * create image converter
	 */
	image_cvt = new imageConverter();

	if( !image_cvt )
	{
		ROS_ERROR("failed to create imageConverter");
		return 0;
	}


	/*
	 * advertise publisher topics
	 */
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "raw", 2, image_pub);
	ROS_CREATE_PUBLISHER(sensor_msgs::CameraInfo, "camera_info", 2, camera_info_pub);


	/*
	 * start the camera streaming
	 */
	if( !stream->Open() )
	{
		ROS_ERROR("failed to start streaming video source");
		return 0;
	}


	/*
	 * start publishing video frames
	 */
	while( ROS_OK() )
	{
		if( !aquireFrame() )
		{
			if( !stream->IsStreaming() )
			{
				ROS_INFO("stream is closed or reached EOS, exiting node...");
				break;
			}
		}

		if( ROS_OK() )
			ROS_SPIN_ONCE();
	}


	/*
	 * free resources
	 */
	delete stream;
	delete image_cvt;

	return 0;
}

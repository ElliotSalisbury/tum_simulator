/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Syncronizes shutters across multiple cameras
 * Author: John Hsu
 * Date: 10 June 2013
 */

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Image.hh"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "elliot_plugins/gazebo_ros_multicamera.h"


cv::Mat BuildPerspProjMat(float fov, float aspect, float znear, float zfar) {
	float cot = tanf(M_PI_2 - (fov / 2.0));
	float zrange = znear - zfar;
	/*
	00 01 02 03
	10 11 12 13
	20 21 22 23
	30 31 32 33
	*/
	cv::Mat perspectiveTransform = cv::Mat::zeros(4,4,CV_32F);
	perspectiveTransform.at<float>(0,0) = cot / aspect;
	perspectiveTransform.at<float>(0,1) = 0;
	perspectiveTransform.at<float>(0,2) = 0;
	perspectiveTransform.at<float>(0,3) = 0;

	perspectiveTransform.at<float>(1,0) = 0;
	perspectiveTransform.at<float>(1,1) = cot;
	perspectiveTransform.at<float>(1,2) = 0;
	perspectiveTransform.at<float>(1,3) = 0;

	perspectiveTransform.at<float>(2,0) = 0;
	perspectiveTransform.at<float>(2,1) = 0;
	perspectiveTransform.at<float>(2,2) = (zfar + znear) / zrange;
	perspectiveTransform.at<float>(2,3) = (2 * zfar * znear) / zrange;

	perspectiveTransform.at<float>(3,0) = 0;
	perspectiveTransform.at<float>(3,1) = 0;
	perspectiveTransform.at<float>(3,2) = -1.0f;
	perspectiveTransform.at<float>(3,3) = 0;

	return perspectiveTransform;
}

cv::Mat BuildRotationMat(float yawa, float pitcha) {
	float yawcos = cos(yawa);
	float yawsin = sin(yawa);

	/*
	00 01 02 03
	10 11 12 13
	20 21 22 23
	30 31 32 33
	*/
	cv::Mat yaw = cv::Mat::zeros(4,4,CV_32F);
	yaw.at<float>(0,0) = yawcos;
	yaw.at<float>(0,1) = 0;
	yaw.at<float>(0,2) = yawsin;
	yaw.at<float>(0,3) = 0;

	yaw.at<float>(1,0) = 0;
	yaw.at<float>(1,1) = 1;
	yaw.at<float>(1,2) = 0;
	yaw.at<float>(1,3) = 0;

	yaw.at<float>(2,0) = -yawsin;
	yaw.at<float>(2,1) = 0;
	yaw.at<float>(2,2) = yawcos;
	yaw.at<float>(2,3) = 0;

	yaw.at<float>(3,0) = 0;
	yaw.at<float>(3,1) = 0;
	yaw.at<float>(3,2) = 0;
	yaw.at<float>(3,3) = 1;

	float pitchcos = cos(pitcha);
	float pitchsin = sin(pitcha);

	cv::Mat pitch = cv::Mat::zeros(4,4,CV_32F);
	pitch.at<float>(0,0) = 1;
	pitch.at<float>(0,1) = 0;
	pitch.at<float>(0,2) = 0;
	pitch.at<float>(0,3) = 0;

	pitch.at<float>(1,0) = 0;
	pitch.at<float>(1,1) = pitchcos;
	pitch.at<float>(1,2) = -pitchsin;
	pitch.at<float>(1,3) = 0;

	pitch.at<float>(2,0) = 0;
	pitch.at<float>(2,1) = pitchsin;
	pitch.at<float>(2,2) = pitchcos;
	pitch.at<float>(2,3) = 0;

	pitch.at<float>(3,0) = 0;
	pitch.at<float>(3,1) = 0;
	pitch.at<float>(3,2) = 0;
	pitch.at<float>(3,3) = 1;

	return pitch * yaw;
}

void display() { /* empty function   required as of glut 3.0 */ }

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMultiCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosMultiCamera::GazeboRosMultiCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosMultiCamera::~GazeboRosMultiCamera()
{
}

void GazeboRosMultiCamera::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  MultiCameraPlugin::Load(_parent, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // initialize shared_ptr members
  this->image_connect_count_ = boost::shared_ptr<int>(new int(0));
  this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
  this->was_active_ = boost::shared_ptr<bool>(new bool(false));

	//check we have 6 cameras
	if (this->camera.size() >= 6) {
		//we want to create the full combined image, we will assume all cameras are equal in width,height and depth
		unsigned int width = this->width[0];
		unsigned int height = this->height[0];
		unsigned int totalWidth = width*4;
		unsigned int totalHeight = height*3;

		this->util = new GazeboRosCameraUtils();
    this->util->parentSensor_ = this->parentSensor;
    this->util->width_   = totalWidth;
    this->util->height_  = totalHeight;
    this->util->depth_   = this->depth[0];
    this->util->format_  = this->format[0];
    this->util->camera_  = this->camera[0];
    // Set up a shared connection counter (this isnt needed now but whatever)
    this->util->image_connect_count_ = this->image_connect_count_;
    this->util->image_connect_count_lock_ = this->image_connect_count_lock_;
    this->util->was_active_ = this->was_active_;
	
		std::string cameraName(this->camera[0]->GetName());
		cameraName = cameraName.substr(cameraName.find_last_of(":")+1);
		cameraName = cameraName.substr(0,cameraName.find("("));
		std::string topicName("/");
		topicName.append(cameraName);
		this->util->Load(_parent, _sdf, topicName, 0.0);

		//FIXME we assume format is 8bit RGB
		this->fullImage_ = cv::Mat::zeros(totalHeight,totalWidth,CV_8UC3);
		//now create the sub images for each camera to copy too
		this->subImages_.push_back(this->fullImage_(cv::Rect(width,height,width,height)));
		this->subImages_.push_back(this->fullImage_(cv::Rect(0,height,width,height)));
		this->subImages_.push_back(this->fullImage_(cv::Rect(width*3,height,width,height)));
		this->subImages_.push_back(this->fullImage_(cv::Rect(width*2,height,width,height)));
		this->subImages_.push_back(this->fullImage_(cv::Rect(width,height*2,width,height)));
		this->subImages_.push_back(this->fullImage_(cv::Rect(width,0,width,height)));
		this->subImages_.push_back(this->fullImage_(cv::Rect(0,0,width,height)));

		this->subImages_[0] = cv::Scalar(0,255,0);
		this->subImages_[1] = cv::Scalar(255,0,0);
		this->subImages_[2] = cv::Scalar(0,0,255);
		this->subImages_[3] = cv::Scalar(255,255,0);
		this->subImages_[4] = cv::Scalar(0,255,255);
		this->subImages_[5] = cv::Scalar(255,0,255);

		//initialize the cube coordinates
		cv::Vec3f cubeArray[8] = {
				cv::Vec3f(1,-1,1),
				cv::Vec3f(1,1,1),
				cv::Vec3f(1,1,-1),
				cv::Vec3f(1,-1,-1),
				cv::Vec3f(-1,-1,1),
				cv::Vec3f(-1,1,1),
				cv::Vec3f(-1,1,-1),
				cv::Vec3f(-1,-1,-1)};
		cv::Vec3f cubeCoordinates[6][4] = {
				{cubeArray[1],cubeArray[5],cubeArray[4],cubeArray[0]},
				{cubeArray[2],cubeArray[1],cubeArray[0],cubeArray[3]},
				{cubeArray[6],cubeArray[2],cubeArray[3],cubeArray[7]},
				{cubeArray[5],cubeArray[6],cubeArray[7],cubeArray[4]},
				{cubeArray[0],cubeArray[4],cubeArray[7],cubeArray[3]},
				{cubeArray[2],cubeArray[6],cubeArray[5],cubeArray[1]} };
		memcpy(this->cubeCoordinates_, cubeCoordinates, sizeof(cubeCoordinates));

		this->perspectiveTransform_ = BuildPerspProjMat(1.5f, 1.0f, 0.1f, 10.0f);

		//Calculate the view rotation matrix
		this->updateCameraAngle(0,1);

	} else {
    ROS_FATAL_STREAM("The 360 camera sensor requires 6 cameras, each facing in different perpendicular directions");
    return;
  }
}

void GazeboRosMultiCamera::updateCameraAngle(float yaw, float pitch) {
		this->cameraDirection_ = cv::Vec3f(1,0,0);
		cv::Mat rotation = BuildRotationMat(yaw, pitch);

		//ROS_FATAL_STREAM("Direction");
		//ROS_FATAL_STREAM("(" << this->cameraDirection_[0] << "," << this->cameraDirection_[1] << "," << this->cameraDirection_[2] << ")");

		//ROS_FATAL_STREAM("ROTATION");
		//ROS_FATAL_STREAM("(" << rotation.at<float>(0,0) << "," << rotation.at<float>(0,1) << "," << rotation.at<float>(0,2) << ")");
		//ROS_FATAL_STREAM("(" << rotation.at<float>(1,0) << "," << rotation.at<float>(1,1) << "," << rotation.at<float>(1,2) << ")");
		//ROS_FATAL_STREAM("(" << rotation.at<float>(2,0) << "," << rotation.at<float>(2,1) << "," << rotation.at<float>(2,2) << ")");

		float w = this->width[0];
		float h = this->height[0];
		float w2 = w/2.0f;
		float h2 = h/2.0f;
		cv::Point2f inputQuad[4];
		cv::Point2f outputQuad[4];
		inputQuad[0] = cv::Point2f( 0,0 );
    inputQuad[1] = cv::Point2f( w,0  );
    inputQuad[2] = cv::Point2f( w,h);
    inputQuad[3] = cv::Point2f( 0,h);

		for(int i=0; i<6; i++){
				ROS_FATAL_STREAM("---" << i << "---");
			for(int j=0; j<4; j++) {
				cv::Mat model(4, 1, CV_32F);
				model.at<float>(0, 0) = this->cubeCoordinates_[i][j][0];
				model.at<float>(1, 0) = this->cubeCoordinates_[i][j][1];
				model.at<float>(2, 0) = this->cubeCoordinates_[i][j][2];
				model.at<float>(3, 0) = 1.0f;

//				ROS_FATAL_STREAM("Model" << model.at<float>(0,0) << "," << model.at<float>(1,0) << "," << model.at<float>(2,0) << "," << model.at<float>(3,0));

				cv::Mat eye = rotation * model;

				ROS_FATAL_STREAM("Eye" << eye.at<float>(0,0) << "," << eye.at<float>(1,0) << "," << eye.at<float>(2,0) << "," << eye.at<float>(3,0));

				cv::Mat clip = this->perspectiveTransform_ * eye;

//				ROS_FATAL_STREAM("clip" << clip.at<float>(0,0) << "," << clip.at<float>(1,0) << "," << clip.at<float>(2,0) << "," << clip.at<float>(3,0));
				
				if(clip.at<float>(3,0) == 0) {
					clip.at<float>(3,0) = 0.00001f;
				}
				clip.at<float>(0,0) /= clip.at<float>(3,0);
				clip.at<float>(1,0) /= clip.at<float>(3,0);
				clip.at<float>(2,0) /= clip.at<float>(3,0);

				if(eye.at<float>(2,0) < 0 ) {
					if(clip.at<float>(0,0) >= -1 && clip.at<float>(0,0) <= 1) {
						if(clip.at<float>(1,0) >= -1 && clip.at<float>(1,0) <= 1) {
							clip.at<float>(0,0) = 2.0 * clip.at<float>(3,0);
							clip.at<float>(1,0) = 2.0 * clip.at<float>(3,0);
						}
					}
				}

				outputQuad[j] = cv::Point2f( w2 + clip.at<float>(0,0)*w2, h2 + clip.at<float>(1,0)*h2);

				ROS_FATAL_STREAM("" << clip.at<float>(0,0) << "," << clip.at<float>(1,0) << "," << clip.at<float>(2,0) << "," << clip.at<float>(3,0));
			}
			this->transforms_[i] = cv::getPerspectiveTransform( inputQuad, outputQuad );
		}

}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMultiCamera::OnNewFrame(const boost::shared_ptr<msgs::ImagesStamped const> &msg) {
  this->util->sensor_update_time_ = this->util->parentSensor_->GetLastUpdateTime();

  if (this->util->parentSensor_->IsActive())
  {
    common::Time cur_time = this->util->world_->GetSimTime();
    if (cur_time - this->util->last_update_time_ >= this->util->update_period_)
    {
			this->subImages_[0] = cv::Scalar(0,255,0);
			this->subImages_[1] = cv::Scalar(255,0,0);
			this->subImages_[2] = cv::Scalar(0,0,255);
			this->subImages_[3] = cv::Scalar(255,255,0);
			this->subImages_[4] = cv::Scalar(0,255,255);
			this->subImages_[5] = cv::Scalar(255,0,255);
			this->subImages_[6] = cv::Scalar(0,0,0);
			for(int i=0; i<msg->image_size(); i++) {
				const msgs::Image image = msg->image(i);
				unsigned int imageSize = image.width() * image.height() * 3; //FIXME get image depth
				unsigned char *workableData = new unsigned char[imageSize];
				memcpy(workableData, image.data().c_str(), imageSize);

				cv::Mat cvImage(image.height(), image.width(), CV_8UC3, workableData);
				//cvImage.copyTo(this->subImages_[i]);

					cv::Mat output;
					cv::warpPerspective(cvImage,output,this->transforms_[i],cvImage.size());
					cv::addWeighted(this->subImages_[6],1.0,output,1.0,0.0,this->subImages_[6]);
					output.copyTo(this->subImages_[i],output);

				delete[] workableData;
			}
			this->util->PutCameraData(this->fullImage_.data);
		  this->util->PublishCameraInfo();
			this->util->last_update_time_ = cur_time;
		}
	}
	static float angle = 0.0f;
	angle += 0.01f;
	this->updateCameraAngle(angle,cos(angle));
}
}

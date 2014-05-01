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
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <elliot_plugins/MultiCameraPlugin.h>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(MultiCameraPlugin)

/////////////////////////////////////////////////
MultiCameraPlugin::MultiCameraPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
MultiCameraPlugin::~MultiCameraPlugin()
{
  this->parentSensor.reset();
  this->camera.clear();
}

/////////////////////////////////////////////////
void MultiCameraPlugin::Load(sensors::SensorPtr _sensor,
  sdf::ElementPtr /*_sdf*/)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "MultiCameraPlugin requires a CameraSensor.\n";
    if (boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
      gzmsg << "It is a depth camera sensor\n";
    if (boost::dynamic_pointer_cast<sensors::CameraSensor>(_sensor))
      gzmsg << "It is a camera sensor\n";
  }

  if (!this->parentSensor)
  {
    gzerr << "MultiCameraPlugin not attached to a camera sensor\n";
    return;
  }

	this->node = transport::NodePtr(new transport::Node());
	this->node->Init();
	this->ImagesSub = this->node->Subscribe<msgs::ImagesStamped>(this->parentSensor->GetTopic(), &MultiCameraPlugin::OnNewFrame, this);

  for (unsigned int i = 0; i < this->parentSensor->GetCameraCount(); ++i)
  {
    this->camera.push_back(this->parentSensor->GetCamera(i));

    // save camera attributes
    this->width.push_back(this->camera[i]->GetImageWidth());
    this->height.push_back(this->camera[i]->GetImageHeight());
    this->depth.push_back(this->camera[i]->GetImageDepth());
    this->format.push_back(this->camera[i]->GetImageFormat());
  }

  this->parentSensor->SetActive(true);
}

void MultiCameraPlugin::OnNewFrame(const boost::shared_ptr<msgs::ImagesStamped const> &msg) {
}

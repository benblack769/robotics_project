/*
 * Copyright 2016 Open Source Robotics Foundation
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


//#include "osrf_gear/ARIAC.hh"
//#include "osrf_gear/LogicalCameraImage.h"

#include <sdf/sdf.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/msgs/logical_camera_image.pb.h"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/transport/TransportTypes.hh"

// ROS
#include <ros/ros.h>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <string>
#include <algorithm>


//own message
#include <ros_pyenv/scan_msg.h>

namespace gazebo{
class CustomizedLogicalCamera : public SensorPlugin
{
public: CustomizedLogicalCamera();

public: virtual ~CustomizedLogicalCamera();

protected: sensors::SensorPtr sensor;

public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

public: void OnImage(ConstLogicalCameraImagePtr &_msg);

protected: transport::NodePtr node;

protected: transport::SubscriberPtr imageSub;

protected: std::string robotNamespace;

protected: ros::NodeHandle *rosnode;

protected: ros::Publisher imagePub;
};

GZ_REGISTER_SENSOR_PLUGIN(CustomizedLogicalCamera);

CustomizedLogicalCamera::CustomizedLogicalCamera()
{
}

CustomizedLogicalCamera::~CustomizedLogicalCamera()
{
  this->rosnode->shutdown();
}
std::string remove_colons(std::string str ){
    str.erase(std::remove(str.begin(), str.end(), ':'), str.end());
    return str;
}
void CustomizedLogicalCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    std::cerr << "bar1 "<< std::endl;
    std::cerr << "bar1 "<< std::endl;
  // load parameters
  this->robotNamespace = "logical_camera";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement(
        "robotNamespace")->Get<std::string>() + "/";
  }

  // Make sure the ROS node for Gazebo has already been initialized
  //if (!ros::isInitialized())
  //{
    //ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
    //    << "unable to load plugin. Load the Gazebo system plugin "
    //    << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
//    return;
  //}
  this->sensor = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();//this->model->GetWorld()->Name());
  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  //this->FindLogicalCamera();
  if (!this->sensor)
  {
    gzerr << "No logical camera found on any link\n";
    return;
  }

  std::string imageTopic_ros = remove_colons(_parent->ScopedName());
  if (_sdf->HasElement("image_topic_ros")) {
    imageTopic_ros = _sdf->Get<std::string>("image_topic_ros");
  }

  this->imageSub = this->node->Subscribe(this->sensor->Topic(),
          &CustomizedLogicalCamera::OnImage, this);
  gzdbg << "Subscribing to gazebo topic: " << this->sensor->Topic() << "\n";

  this->imagePub = this->rosnode->advertise<ros_pyenv::scan_msg>(imageTopic_ros, 1, true);
  gzdbg << "Publishing to ROS topic: " << imagePub.getTopic() << "\n";
}

void CustomizedLogicalCamera::OnImage(ConstLogicalCameraImagePtr &_msg)
{
  ros_pyenv::scan_msg imageMsg;
  msgs::Vector3d cameraPosition = _msg->pose().position();
  //msgs::Quaternion cameraOrientation = _msg->pose().orientation();
  imageMsg.camera_pos.x = cameraPosition.x();
  imageMsg.camera_pos.y = cameraPosition.y();
  imageMsg.camera_pos.z = cameraPosition.z();

  for (int i = 0; i < _msg->model_size(); ++i)
  {
    msgs::Vector3d position = _msg->model(i).pose().position();
    //msgs::Quaternion orientation = _msg->model(i).pose().orientation();
    ros_pyenv::named_position modelMsg;
    modelMsg.pos.x = position.x();
    modelMsg.pos.y = position.y();
    modelMsg.pos.z = position.z();
    modelMsg.name = _msg->model(i).name();//ariac::DetermineModelType(_msg->model(i).name());
    imageMsg.sensed_objs.push_back(modelMsg);
  }
  this->imagePub.publish(imageMsg);
}
}

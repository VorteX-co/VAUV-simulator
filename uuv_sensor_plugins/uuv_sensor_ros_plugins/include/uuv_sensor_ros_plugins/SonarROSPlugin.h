// Copyright 2022 VorteX-co
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef __UUV_SONAR_ROS_PLUGIN_HH__
#define __UUV_SONAR_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <rclcpp/rclcpp.hpp>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.h>

/***************************************
 * Gazebo plugin for controlling the speed of rotation of the
 * Ping360 Scanning Imaging Sonar beam
 **************************************/

namespace gazebo
{
  class SonarROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: SonarROSPlugin();

    /// \brief Class destructor
    public: ~SonarROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: void SetVelocity(const double &_vel);

      /// \brief Pointer to the Sonar model.
   protected: physics::ModelPtr model_;

      /// \brief Pointer to the Sonar-Beam joint.
   protected: physics::JointPtr joint_;

      /// \brief A PID controller for the Beam joint.
   protected: common::PID pid_;

  };
}

#endif // __UUV_SONAR_ROS_PLUGIN_HH__

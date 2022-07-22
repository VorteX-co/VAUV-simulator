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

#include <uuv_sensor_ros_plugins/SonarROSPlugin.h>
#include <string>
#include <algorithm>
#include <limits>
#include <memory>


namespace gazebo
{
/////////////////////////////////////////////////
SonarROSPlugin::SonarROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
SonarROSPlugin::~SonarROSPlugin()
{ }

/////////////////////////////////////////////////
void SonarROSPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);
  // Store the model pointer for convenience.
  this->model_ = _model;
  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  // this->joint = _model->GetJoints()[1];
  this->joint_ = _model->GetJoint("swift/sonar_beam_joint");


  // Setup a P-controller, with a gain of 0.1.
  this->pid_ = common::PID(0.1, 0.0, 0.0);

  // Apply the P-controller to the joint.
  this->model_->GetJointController()->SetVelocityPID(
      this->joint_->GetScopedName(), this->pid_);

  // Default to zero velocity
  double velocity =0;

  // Check that the velocity element exists, then read the value
  if (_sdf->HasElement("velocity"))
    velocity = _sdf->Get<double>("velocity");

  this->SetVelocity(velocity);

}
/// \brief Set the velocity of the Ping360
/// \param[in] _vel New target velocity
void SonarROSPlugin::SetVelocity(const double &_vel)
{
  // Set the joint's target velocity.
  this->model_->GetJointController()->SetVelocityTarget(
      this->joint_->GetScopedName(), _vel);
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(SonarROSPlugin)
}

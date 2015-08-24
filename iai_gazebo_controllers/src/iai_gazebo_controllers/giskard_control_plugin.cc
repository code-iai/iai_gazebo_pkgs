/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Andrei Haidu, Georg Bartels
 *  Institute for Artificial Intelligence, Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include "iai_gazebo_controllers/giskard_control_plugin.hh"
#include "iai_gazebo_controllers/gazebo_utils.hh"

using namespace iai_gazebo_controllers;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GiskardControlPlugin)

//////////////////////////////////////////////////

GiskardControlPlugin::GiskardControlPlugin()
{
}

//////////////////////////////////////////////////

GiskardControlPlugin::~GiskardControlPlugin()
{
}

//////////////////////////////////////////////////

void GiskardControlPlugin::Load(physics::WorldPtr world, sdf::ElementPtr self_description)
{
  InitInternals(world, self_description);

  InitGazeboCommunication();
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitInternals(gazebo::physics::WorldPtr world, sdf::ElementPtr self_description)
{
  world_ = world;
  self_description_ = self_description;

  InitControlledModel();
  InitObservedModel();
  InitController();
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitControlledModel()
{
  std::string controlledModelName;
  assert(GetSDFValue("controlledModel", self_description_, controlledModelName));
  controlled_model_ = world_->GetModel(controlledModelName);
  assert(controlled_model_.get());
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitObservedModel()
{
  std::string model_name;
  assert(GetSDFValue("observedModel", self_description_, model_name));
  observed_model_ = world_->GetModel(model_name);
  assert(observed_model_.get());
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitController()
{
  ReadMotionDescriptions();
  controlled_model_->SetGravityMode(false);
  // TODO: get this number from somewhere
  assert(controller_.start(GetObservables(), 10));
}

//////////////////////////////////////////////////

void GiskardControlPlugin::ReadMotionDescriptions()
{
  std::string motion_file;
  assert(GetSDFValue("motionFile", self_description_, motion_file));

  YAML::Node node = YAML::LoadFile(motion_file);
  controller_ = giskard::generate(node.as<giskard::QPControllerSpec>());
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitGazeboCommunication()
{
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GiskardControlPlugin::UpdateCallback, this, _1));
}

//////////////////////////////////////////////////

void GiskardControlPlugin::UpdateCallback(const common::UpdateInfo& info)
{
  gazebo::physics::LinkPtr link = controlled_model_->GetLinks()[0];
  
  // TODO: get this number from somewhere
  // TODO: to sth smarter than just dying
  assert(controller_.update(GetObservables(), 10));

//std::cout << "\n\nsoft-expressions:\n";
//for(size_t i=0; i<controller_.get_soft_expressions().size(); ++i)
//  std::cout << controller_.get_soft_expressions()[i]->value() << "\n";
//std::cout << "\nlower bounds:\n";
//for(size_t i=0; i<controller_.get_soft_lower_bounds().size(); ++i)
//  std::cout << controller_.get_soft_lower_bounds()[i]->value() << "\n";
//std::cout << "\nupper bounds:\n";
//for(size_t i=0; i<controller_.get_soft_upper_bounds().size(); ++i)
//  std::cout << controller_.get_soft_upper_bounds()[i]->value() << "\n";
//std::cout << "\nweights:\n";
//for(size_t i=0; i<controller_.get_soft_weights().size(); ++i)
//  std::cout << controller_.get_soft_weights()[i]->value() << "\n";
//
//std::cout << "\n\n";
//
//using Eigen::operator<<;
//std::cout << "A_:\n" << controller_.get_qp_builder().get_A() << "\n";
//std::cout << "H_:\n" << controller_.get_qp_builder().get_H() << "\n";

  Eigen::VectorXd command = controller_.get_command();
  // TODO: make helper function for this
  link->SetLinearVel(gazebo::math::Vector3(command(0), command(1), command(2)));
  link->SetAngularVel(gazebo::math::Vector3(command(3), command(4), command(5)));
}

//////////////////////////////////////////////////

Eigen::VectorXd GiskardControlPlugin::GetObservables()
{
  Eigen::VectorXd result(12);

  result.segment(0, 6) = PoseToGiskardInputs(controlled_model_->GetLinks()[0]->GetWorldPose());
  result.segment(6, 6) = PoseToGiskardInputs(observed_model_->GetLinks()[0]->GetWorldPose());

  return result;
}

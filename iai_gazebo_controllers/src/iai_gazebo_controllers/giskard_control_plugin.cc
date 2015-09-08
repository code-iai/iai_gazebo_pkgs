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
#include <iai_gazebo_controllers/yaml_parsing.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/util/LogRecord.hh>

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

  delay_sim_start_thread_ = boost::shared_ptr<boost::thread>(
      new boost::thread(&GiskardControlPlugin::DelaySimStart, this));
}

//////////////////////////////////////////////////

void GiskardControlPlugin::UpdateCallback(const common::UpdateInfo& info)
{
  // delaying start of first motion
  if(world_->GetSimTime().Double() <= move_start_delay_)
  {
    SetCommand(Eigen::VectorXd::Zero(6), false);
    return;
  }

  if(MotionFinished())
    if(controller_specs_.empty())
    {
      if(!requested_shutdown_)
      {
        StopLogging();
        RequestGazeboShutdown();  
        requested_shutdown_ = true;
      }
    }
    else
      InitNextController();

  // TODO: get this number from somewhere
  // TODO: to sth smarter than just dying
  assert(controller_.update(GetObservables(), 10));

  SetCommand(GetControlledJacobian().data * controller_.get_command());
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitInternals(gazebo::physics::WorldPtr world, sdf::ElementPtr self_description)
{
  world_ = world;
  self_description_ = self_description;
  
  requested_shutdown_ = false;
  ReadExperimentSpec();
  InitNextController();
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitNextController()
{
  assert(controller_specs_.size() > 0);
  controller_ = giskard::generate(controller_specs_[0]);
  scope_ = giskard::generate(controller_specs_[0].scope_);
  // TODO: get this string from somewhere
  controlled_frame_ = scope_.find_frame_expression("mug-frame");
  controller_specs_.erase(controller_specs_.begin());

  cmd_buffer_.clear();
  max_cmd_buffer_size_ = max_cmd_buffer_sizes_[0];
  max_cmd_buffer_sizes_.erase(max_cmd_buffer_sizes_.begin());

  // TODO: get this number from somewhere
  assert(controller_.start(GetObservables(), 10));

  std::cout << "SWITCHED CONTROLLERS" << std::endl;
}

//////////////////////////////////////////////////

void GiskardControlPlugin::ReadExperimentSpec()
{
  // getting file name out of SDF
  std::string experiment_file;
  assert(GetSDFValue("experimentFile", self_description_, experiment_file));

  // parsing experiment specs
  YAML::Node node = YAML::LoadFile(experiment_file);
  ExperimentSpec exp_spec = node.as< ExperimentSpec >();

  // making sure move-start-delay is valid
  move_start_delay_ = exp_spec.move_start_delay_;
  assert(move_start_delay_ >= 0.0);

  // making sure sim-start-delay is valid
  sim_start_delay_ = exp_spec.sim_start_delay_;
  assert(sim_start_delay_ >= 0.0);

  // making sure controlled model is valid
  controlled_model_ = world_->GetModel(exp_spec.controlled_model_);
  assert(controlled_model_.get());
  controlled_model_->SetGravityMode(false);

  // making sure observed model is valid
  observed_model_ = world_->GetModel(exp_spec.observed_model_);
  assert(observed_model_.get());

  // making sure we got at least one controller spec
  assert(exp_spec.controller_specs_.size() > 0);

  // parsing all controller specs
  for(size_t i=0; i<exp_spec.controller_specs_.size(); ++i)
  {
    node = YAML::LoadFile(exp_spec.controller_specs_[i].controller_file_);
    
    // parsing controller spec and making sure it compiles
    giskard::QPControllerSpec controller_spec = node.as<giskard::QPControllerSpec>();
    giskard::generate(controller_spec);
    controller_specs_.push_back(controller_spec);

    max_cmd_buffer_sizes_.push_back(exp_spec.controller_specs_[i].max_cmd_buffer_size_);
  }
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitGazeboCommunication()
{
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GiskardControlPlugin::UpdateCallback, this, _1));

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init(world_->GetName());
  serverControlPublisher_ = node->Advertise<msgs::ServerControl>("/gazebo/server/control");
  visualizationPublisher_ = node->Advertise<msgs::Vector3d>("/giskard/visualization", 1);
}

//////////////////////////////////////////////////

bool GiskardControlPlugin::MotionFinished() const
{
  assert(max_cmd_buffer_size_ > 0);

  if(cmd_buffer_.size() != max_cmd_buffer_size_)
    return false;

  double min_trans_vel = 0.01;
  double min_ang_vel = 0.02;

  for(std::deque<Eigen::VectorXd>::const_iterator it = cmd_buffer_.begin(); it!=cmd_buffer_.end(); ++it)
    for(size_t row = 0; row < it->rows(); ++row)
      // TODO: sth more sophisticated here, please
      // TODO: get this number from somewhere
      if((0<row) && (row<3) && (std::abs(it->operator()(row)) > min_trans_vel))
        return false;
      else if ((3<row) && (row<6) && (std::abs(it->operator()(row)) > min_ang_vel))
        return false;

  return true;
}

//////////////////////////////////////////////////

void GiskardControlPlugin::DelaySimStart()
{
  world_->SetPaused(true);
  std::cout << "Paused the world, starting sim in " << sim_start_delay_ << " sec.." << std::endl;
  usleep(sim_start_delay_ * 1000000);
  world_->SetPaused(false);
//  usleep(logDelay * 1000000);
  StartLogging();
}

//////////////////////////////////////////////////

void GiskardControlPlugin::StartLogging()
{
  util::LogRecord::Instance()->SetBasePath("logs");
  util::LogRecord::Instance()->Start("txt");
}

//////////////////////////////////////////////////

void GiskardControlPlugin::StopLogging()
{
  util::LogRecord::Instance()->Stop();
}

//////////////////////////////////////////////////

void GiskardControlPlugin::RequestGazeboShutdown()
{
  std::cout << "REQUESTING GAZEBO SERVER SHUTDOWN" << std::endl;
  msgs::ServerControl server_msg;
  server_msg.set_stop(true);
  serverControlPublisher_->Publish(server_msg);
}

//////////////////////////////////////////////////

Eigen::VectorXd GiskardControlPlugin::GetObservables()
{
  Eigen::VectorXd result(12);

  result.segment(0, 6) = PoseToGiskardInputs(controlled_model_->GetLinks()[0]->GetWorldPose());
  result.segment(6, 6) = PoseToGiskardInputs(observed_model_->GetLinks()[0]->GetWorldPose());

  return result;
}

//////////////////////////////////////////////////

KDL::Jacobian GiskardControlPlugin::GetControlledJacobian()
{
  controlled_frame_->setInputValues(toSTL(GetObservables()));
  controlled_frame_->value();
  
  KDL::Jacobian jac(6);
  for(size_t i=0; i<6; ++i)
    jac.setColumn(i, controlled_frame_->derivative(i));
 
  return jac;
}

//////////////////////////////////////////////////

void GiskardControlPlugin::SetCommand(const Eigen::VectorXd& command, bool with_logging)
{
  controlled_model_->GetLinks()[0]->SetLinearVel(gazebo::math::Vector3(command(0), command(1), command(2)));
  controlled_model_->GetLinks()[0]->SetAngularVel(gazebo::math::Vector3(command(3), command(4), command(5)));

  // remembering command
  if(with_logging)
  {
    assert(max_cmd_buffer_size_ > 0);
    if(cmd_buffer_.size() >= max_cmd_buffer_size_)
      cmd_buffer_.pop_back();

    cmd_buffer_.push_front(command);
  }
}

//////////////////////////////////////////////////

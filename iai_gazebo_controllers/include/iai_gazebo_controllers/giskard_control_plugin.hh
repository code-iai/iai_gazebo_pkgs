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

#ifndef GISKARD_CONTROL_PLUGIN_HH
#define GISKARD_CONTROL_PLUGIN_HH

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>

#include "iai_gazebo_controllers/gazebo_utils.hh"
#include <giskard/giskard.hpp>
#include <deque>
#include <vector>

namespace iai_gazebo_controllers
{
  class GiskardControlPlugin : public gazebo::WorldPlugin
  {
    public:
      GiskardControlPlugin();
      virtual ~GiskardControlPlugin();

    protected:
      // main interface of this plugin
      virtual void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr self_description);
      void UpdateCallback(const gazebo::common::UpdateInfo& info);

      // inits
      void InitInternals(gazebo::physics::WorldPtr world, sdf::ElementPtr self_description);
      void InitControlledModel();
      void InitObservedModel();
      void ReadExperimentSpec();
      void InitNextController();
      void InitGazeboCommunication();

      // state machine
      bool MotionFinished() const;
      void DelaySimStart();

      void StartLogging();
      void StopLogging();
      void RequestGazeboShutdown();

      // control helpers
      Eigen::VectorXd GetObservables();
      KDL::Jacobian GetControlledJacobian();
      void SetCommand(const Eigen::VectorXd& command, bool with_logging=true);

      // data we got upon init
      gazebo::physics::WorldPtr world_;
      sdf::ElementPtr self_description_;

      // internals of controller
      gazebo::physics::ModelPtr controlled_model_, observed_model_;
      std::vector<giskard::QPControllerSpec> controller_specs_;
      std::vector<size_t> max_twist_buffer_sizes_;
      std::vector<double> min_angular_vel_thresholds_;
      std::vector<double> min_translational_vel_thresholds_;
      giskard::QPController controller_;
      std::deque<iai_gazebo_controllers::Twist> twist_buffer_;
      size_t max_twist_buffer_size_;
      double min_angular_vel_threshold_, min_translational_vel_threshold_;
      double sim_start_delay_, move_start_delay_, log_delay_;

      KDL::Expression<KDL::Frame>::Ptr controlled_frame_;
      giskard::Scope scope_;

      // gazebo communication and infrastructure
      gazebo::event::ConnectionPtr updateConnection_;
      gazebo::transport::PublisherPtr serverControlPublisher_, visualizationPublisher_, logControlPublisher_;
      boost::shared_ptr<boost::thread> delay_sim_start_thread_;
      bool requested_shutdown_;
      bool shutdown_;
  };
}
#endif // GISKARD_CONTROL_PLUGIN_HH

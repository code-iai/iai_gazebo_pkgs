/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Andrei Haidu,
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

#ifndef WORLD_CONTROL_PLUGIN_HH
#define WORLD_CONTROL_PLUGIN_HH

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>

#include <fccl/control/CartesianConstraintController.h>

#include <iai_gazebo_controllers/motion_description.hh>
#include <iai_gazebo_controllers/gazebo_utils.hh>

#include <vector>

namespace iai_gazebo_controllers
{

  #define DELTA_DERIVATIVE 0.01
  #define DEFAULT_CYCLE_TIME 0.001

  class WorldControlPlugin : public gazebo::WorldPlugin
  {
    public:
      WorldControlPlugin();
      virtual ~WorldControlPlugin();

    protected:
      virtual void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    private:
      // POINTERS TO GAZEBO STRUCTURES
      gazebo::physics::WorldPtr self_;
      sdf::ElementPtr self_description_;
      gazebo::physics::ModelPtr controlled_model_;

      // GAZEBO COMMUNICATION
      gazebo::event::ConnectionPtr updateConnection_;
      gazebo::transport::PublisherPtr serverControlPublisher_;

      // VARIABLES OF POURING CONTROLLER
      std::vector<MotionDescription> motions_;
      fccl::control::CartesianConstraintController controller_;
      fccl::utils::TransformMap transforms_;
      double simulation_start_delay_;
      gazebo::common::Time last_control_time_, accumulated_convergence_time_;
      unsigned int current_motion_index_;

      // CALLBACKS
      void UpdateCallback(const gazebo::common::UpdateInfo& info);

      // INIT FUNCTIONS
      void DelaySimulationStart();
      void GetControlledModel();
      void GetStartDelay();
      void SetupConnections();
      void ReadMotionDescriptions();
      void InitController(const std::vector<MotionDescription>& motions, unsigned int index);
 
      // AUX FUNCTIONS OF CONSTRAINT CONTROLLER
      void PerformVelocityControl(const Twist& twist);
      void FillTransformMap();
      void SwitchToNextMotionPhase();
      bool CurrentMotionPhaseOver() const;
      bool MoreMotionPhasesRemaining() const;
 
      // GAZEBO AUS FUNCTIONS
      gazebo::common::Time GetCycleTime(double default_cycle_time) const;
      void StartLogging();
      void StopLogging();
      void RequestGazeboShutdown();
      bool SimulationStartDelayOver() const;
 
      // \brief Thread for checking the start delay of the simulation
      boost::thread* checkStartDelay;
      // \brief Delay timer
      double startDelay;
  };
}
#endif

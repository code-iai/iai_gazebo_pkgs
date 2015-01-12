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

#include "iai_gazebo_controllers/WorldControlPlugin.hh"
#include "iai_gazebo_controllers/gazebo_utils.hh"

using namespace iai_gazebo_controllers;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(WorldControlPlugin)

//////////////////////////////////////////////////
WorldControlPlugin::WorldControlPlugin()
{
}

//////////////////////////////////////////////////
WorldControlPlugin::~WorldControlPlugin()
{
}

//////////////////////////////////////////////////
void WorldControlPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  self_ = _parent;
  
  GetSDFValue("startDelay", _sdf, startDelay, 0.0);
  assert(startDelay >= 0.0);
  
  GetControlledModel(_sdf);
 
  // start thread which delays start of simulation
  checkStartDelay = new boost::thread(&WorldControlPlugin::DelaySimulationStart, this);
}

//////////////////////////////////////////////////
void WorldControlPlugin::DelaySimulationStart()
{
  self_->SetPaused(true);
  std::cout << "Paused the world, starting sim in " <<  startDelay << " sec..\n";
  usleep(startDelay * 1000000);
  self_->SetPaused(false);
}

void WorldControlPlugin::GetControlledModel(sdf::ElementPtr _sdf)
{
  std::string controlledModelName;
  assert(GetSDFValue("controlledModel", _sdf, controlledModelName));
  controlled_model_ = self_->GetModel(controlledModelName);
  assert(controlled_model_);
}

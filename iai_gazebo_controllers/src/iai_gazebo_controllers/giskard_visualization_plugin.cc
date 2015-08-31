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

#include <iai_gazebo_controllers/giskard_visualization_plugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>

using namespace iai_gazebo_controllers;

GZ_REGISTER_VISUAL_PLUGIN(GiskardVisualizationPlugin)

void GiskardVisualizationPlugin::Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr description)
{
 // TODO: implement me

//  update_connection_ = 
//      gazebo::event::Events::ConnectRender(boost::bind(&GiskardVisualizationPlugin::OnUpdate, this));

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  visualization_sub_ = node->Subscribe<gazebo::msgs::Vector3d>("/giskard/visualization", 
      &GiskardVisualizationPlugin::Callback, this);
}

void GiskardVisualizationPlugin::OnUpdate()
{
  // TODO: implement me
  Ogre::SceneManager* scene_manager = gazebo::gui::get_active_camera()->GetScene()->GetManager();
  if(!scene_manager->hasSceneNode("GiskardVisualization"))
  {
    Ogre::SceneNode* scene_node = scene_manager->getRootSceneNode()->createChildSceneNode("GiskardVisualization");
    scene_node->setVisible(true);
    scene_node->setScale(0.0002, 0.0002, 0.0002);

    Ogre::Entity* entity = scene_manager->createEntity("RimPoint", Ogre::SceneManager::PT_SPHERE);
    entity->setMaterialName("Gazebo/Red");
    entity->setVisible(true);

    scene_node->attachObject(entity);
  }
}

void GiskardVisualizationPlugin::Callback(ConstVector3dPtr& msg)
{
  Ogre::SceneManager* scene_manager = gazebo::gui::get_active_camera()->GetScene()->GetManager();

  if(!scene_manager->hasSceneNode("GiskardVisualization"))
  {
    Ogre::SceneNode* scene_node = scene_manager->getRootSceneNode()->createChildSceneNode("GiskardVisualization");
    scene_node->setVisible(true);
    scene_node->setScale(0.0002, 0.0002, 0.0002);

    Ogre::Entity* entity = scene_manager->createEntity("RimPoint", Ogre::SceneManager::PT_SPHERE);
    entity->setMaterialName("Gazebo/Red");
    entity->setVisible(true);

    scene_node->attachObject(entity);
  }

  scene_manager->getSceneNode("GiskardVisualization")->setPosition(msg->x(), msg->y(), msg->z());
}

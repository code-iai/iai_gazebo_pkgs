#include <iai_gazebo_controllers/constraint_pouring_controller.hh>
#include <iai_gazebo_controllers/gazebo_utils.hh>
#include <boost/bind.hpp>

using namespace gazebo;

namespace iai_gazebo_controllers
{  
  void ConstraintPouringController::Load(physics::WorldPtr parent, sdf::ElementPtr self)
  {
    printf("Loading ConstraintPouringController...\n");
    this->world_ = parent;
    this->self_description_ = self;

    ReadPluginParameters();

    InitController();

    SetupConnections();
  }

  void ConstraintPouringController::UpdateCallback(const common::UpdateInfo& info)
  {
    PerformVelocityControl(Twist(math::Vector3(), math::Vector3()));
  }

  void ConstraintPouringController::InitController()
  {
    controlled_model_->SetGravityMode(false);
  }

  void ConstraintPouringController::ReadPluginParameters()
  {
    std::string controlled_model;
    GetSDFValue("controlled_model", self_description_, controlled_model);
    controlled_model_ = world_->GetModel(controlled_model);
    if(!controlled_model_)
      printf("ERROR: Model '%s' not in world!", controlled_model.c_str());
  }

  void ConstraintPouringController::SetupConnections()
  {
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ConstraintPouringController::UpdateCallback, this, _1));
  }

  void ConstraintPouringController::PerformVelocityControl(const Twist& twist)
  {
    gazebo::physics::LinkPtr link = controlled_model_->GetLinks()[0];
    link->SetLinearVel(twist.linear_velocity_);
    link->SetAngularVel(twist.angular_velocity_);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ConstraintPouringController)
}

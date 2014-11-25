#include <iai_gazebo_controllers/constraint_pouring_controller.hh>
#include <iai_gazebo_controllers/gazebo_utils.hh>
#include <boost/bind.hpp>

using namespace gazebo;

namespace iai_gazebo_controllers
{  
  void ConstraintPouringController::Load(physics::ModelPtr parent, sdf::ElementPtr self)
  {
    printf("Loading ConstraintPouringController...\n");
    this->self_description_ = self;
    this->controlled_model_ = parent;

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
  GZ_REGISTER_MODEL_PLUGIN(ConstraintPouringController)
}

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

    ReadParameters();

    SetupConnections();
  }

  void ConstraintPouringController::UpdateCallback(const common::UpdateInfo& info)
  {
    // TODO(Georg): add constraint controller here
    VelocityControlLink(math::Vector3(), math::Vector3(), 
        controlled_model_->GetLinks()[0]);
  }

  void ConstraintPouringController::ReadParameters()
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

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ConstraintPouringController)
}

#include <iai_gazebo_controllers/constraint_pouring_controller.hh>
#include <iai_gazebo_controllers/gazebo_utils.hh>
#include <boost/bind.hpp>

namespace gazebo
{  
  void ConstraintPouringController::Load(physics::WorldPtr parent, sdf::ElementPtr self)
  {
    printf("Loading ConstraintPouringController...\n");
    this->world_ = parent;
    this->self_model_ = self;

    ReadParameters();

    SetupConnections();
  }

  void ConstraintPouringController::UpdateCallback(const common::UpdateInfo& info)
  {
   // implement me
  }

  void ConstraintPouringController::ReadParameters()
  {
    std::string controlled_model;
    GetSDFValue("controlled_model", self_model_, controlled_model);
    printf("Extracted the following parameter: %s\n", controlled_model.c_str());
  }

  void ConstraintPouringController::SetupConnections()
  {
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ConstraintPouringController::UpdateCallback, this, _1));
  }

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ConstraintPouringController)
}

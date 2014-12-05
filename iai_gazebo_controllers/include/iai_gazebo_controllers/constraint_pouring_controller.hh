#ifndef IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 
#define IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iai_gazebo_controllers/gazebo_utils.hh>

#include <fccl/control/CartesianConstraintController.h>

namespace iai_gazebo_controllers
{
  class ConstraintPouringController : public gazebo::ModelPlugin
  {
    public:
       void Load(gazebo::physics::ModelPtr self, sdf::ElementPtr self_description);

    private: 
      // communication stuff
      gazebo::event::ConnectionPtr updateConnection_;

      // pointers to ourself
      gazebo::physics::ModelPtr self_;
      sdf::ElementPtr self_description_;

      // Cartesian Constraint Controller
      fccl::control::CartesianConstraintController controller_;
      fccl::utils::TransformMap transforms_;

      // callback functions
      void UpdateCallback(const gazebo::common::UpdateInfo& info);

      // init functions
      void InitController();
      void ReadPluginParameters();
      void SetupConnections();

      // control functions
      void PerformVelocityControl(const Twist& twist);

      // aux functions
      void FillTransformMap();
      fccl::base::ConstraintArray GetConstraints();
  };
} // namespace gazebo
#endif //IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 

#ifndef IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 
#define IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace iai_gazebo_controllers
{
  void VelocityControlLink(const gazebo::math::Vector3& linear_velocity,
      const gazebo::math::Vector3& angular_velocity,
      gazebo::physics::Link& Link);

  class ConstraintPouringController : public gazebo::WorldPlugin
  {
    public:
       void Load(gazebo::physics::WorldPtr parent, sdf::ElementPtr self);

       void UpdateCallback(const gazebo::common::UpdateInfo& info);

    private: 
      // communication stuff
      gazebo::event::ConnectionPtr updateConnection_;

      // internal data structures
      gazebo::physics::WorldPtr world_;
      sdf::ElementPtr self_description_;
      gazebo::physics::ModelPtr controlled_model_;

      gazebo::common::PID pid_;

      // internal helper functions
      void ReadParameters();
      void SetupConnections();
  };
} // namespace gazebo
#endif //IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 

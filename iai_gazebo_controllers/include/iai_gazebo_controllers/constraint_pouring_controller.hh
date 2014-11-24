#ifndef IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 
#define IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace iai_gazebo_controllers
{
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
      sdf::ElementPtr self_model_;

      // internal helper functions
      void ReadParameters();
      void SetupConnections();
  };
} // namespace gazebo
#endif //IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class ConstraintPouringController : public WorldPlugin
  {
    public:
       void Load(physics::WorldPtr parent, sdf::ElementPtr self);

       void UpdateCallback(const common::UpdateInfo& info);

    private: 
      // communication stuff
      event::ConnectionPtr updateConnection_;

      // internal data structures
      physics::WorldPtr world_;
      sdf::ElementPtr self_model_;

      // internal helper functions
      void ReadParameters();
      void SetupConnections();
  };
} // namespace gazebo

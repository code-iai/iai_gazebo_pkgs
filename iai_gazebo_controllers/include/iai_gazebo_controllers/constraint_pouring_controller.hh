#ifndef IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 
#define IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iai_gazebo_controllers/gazebo_utils.hh>

#include <fccl/control/CartesianConstraintController.h>

namespace iai_gazebo_controllers
{
  class MotionDescription 
  { 
    public:
      std::string name_;
      double start_delay_;
      fccl::base::ConstraintArray constraints_;
  };

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
      double simulation_start_delay_;
      std::vector<MotionDescription> motions_;
      fccl::control::CartesianConstraintController controller_;
      fccl::utils::TransformMap transforms_;
      gazebo::common::Time last_control_time_;
      unsigned int current_motion_index_;

      // callback functions
      void UpdateCallback(const gazebo::common::UpdateInfo& info);

      // init functions
      void InitController(const std::vector<MotionDescription>& motions, unsigned int index);
      void ReadMotionDescriptions();
      void SetupConnections();

      // control functions
      void PerformVelocityControl(const Twist& twist);

      // aux functions
      void FillTransformMap();
      gazebo::common::Time getCurrentSimTime() const;
      gazebo::common::Time getCycleTime(double default_cycle_time) const;
      bool simulationStartDelayOver() const;
  };
} // namespace gazebo
#endif //IAI_GAZEBO_CONTROLLERS_CONSTRAINT_POURING_CONTROLLER_HH 

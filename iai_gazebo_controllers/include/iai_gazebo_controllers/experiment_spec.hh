#ifndef IAI_GAZEBO_CONTROLLERS_EXPERIMENT_SPEC_HH 
#define IAI_GAZEBO_CONTROLLERS_EXPERIMENT_SPEC_HH 

#include <string>
#include <vector>

namespace iai_gazebo_controllers
{
  class ControllerSpec
  {
    public:
      std::string controller_file_;
      size_t max_twist_buffer_size_;
      double min_angular_vel_threshold_, min_translational_vel_threshold_;
  };

  class ExperimentSpec
  { 
    public:
      double sim_start_delay_, move_start_delay_, log_delay_;
      std::string controlled_model_, observed_model_;
      std::vector<ControllerSpec> controller_specs_;
  };
}

#endif //IAI_GAZEBO_CONTROLLERS_EXPERIMENT_SPEC_HH 

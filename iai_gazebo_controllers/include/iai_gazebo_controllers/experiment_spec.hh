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
      size_t max_cmd_buffer_size_;
  };

  class ExperimentSpec
  { 
    public:
      double sim_start_delay_;
      double move_start_delay_;
      std::vector<ControllerSpec> controller_specs_;
  };
}

#endif //IAI_GAZEBO_CONTROLLERS_EXPERIMENT_SPEC_HH 

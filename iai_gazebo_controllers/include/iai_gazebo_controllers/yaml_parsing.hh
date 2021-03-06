#ifndef IAI_GAZEBO_CONTROLLERS_YAML_PARSING_HH 
#define IAI_GAZEBO_CONTROLLERS_YAML_PARSING_HH 

#include <yaml-cpp/yaml.h>
#include <iai_gazebo_controllers/experiment_spec.hh>

namespace YAML
{  
  inline bool is_controller_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 4) && node["controller-file"] &&
        node["controller-file"].IsScalar() && node["max-twist-buffer-size"] &&
        node["max-twist-buffer-size"].IsScalar() && node["min-angular-vel-threshold"] &&
        node["min-angular-vel-threshold"].IsScalar() && node["min-translational-vel-threshold"] &&
        node["min-translational-vel-threshold"].IsScalar();
  }

  inline bool is_experiment_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 6) && node["sim-start-delay"] &&
        node["sim-start-delay"].IsScalar() && node["move-start-delay"] && 
        node["move-start-delay"].IsScalar() && node["log-delay"] &&
        node["log-delay"].IsScalar() && node["controlled-model"] &&
        node["controlled-model"].IsScalar() && node["observed-model"] && 
        node["observed-model"].IsScalar() && node["controller-specs"] &&
        node["controller-specs"].IsSequence();
  }

  template<> 
  struct convert<iai_gazebo_controllers::ControllerSpec>  
  { 
     
    static Node encode(const iai_gazebo_controllers::ControllerSpec& rhs)
    { 
      Node node; 
      node["controller-file"] = rhs.controller_file_;
      node["max-twist-buffer-size"] = rhs.max_twist_buffer_size_;
      node["min-angular-vel-threshold"] = rhs.min_angular_vel_threshold_;
      node["min-translational-vel-threshold"] = rhs.min_translational_vel_threshold_;
      return node; 
    } 
   
    static bool decode(const Node& node, iai_gazebo_controllers::ControllerSpec& rhs)
    { 
      if(!is_controller_spec(node)) 
        return false; 
   
      rhs.controller_file_ = node["controller-file"].as<std::string>();
      rhs.max_twist_buffer_size_ = node["max-twist-buffer-size"].as<size_t>();
      rhs.min_angular_vel_threshold_ = node["min-angular-vel-threshold"].as<double>();
      rhs.min_translational_vel_threshold_ = node["min-translational-vel-threshold"].as<double>();

      return true; 
    } 
  };

  template<> 
  struct convert<iai_gazebo_controllers::ExperimentSpec>  
  { 
     
    static Node encode(const iai_gazebo_controllers::ExperimentSpec& rhs)
    { 
      Node node; 
      node["controller-specs"] = rhs.controller_specs_;
      node["sim-start-delay"] = rhs.sim_start_delay_;
      node["move-start-delay"] = rhs.move_start_delay_;
      node["log-delay"] = rhs.log_delay_;
      node["controlled-model"] = rhs.controlled_model_;
      node["observed-model"] = rhs.observed_model_;

      return node; 
    } 
   
    static bool decode(const Node& node, iai_gazebo_controllers::ExperimentSpec& rhs)
    { 
      if(!is_experiment_spec(node)) 
        return false; 
   
      rhs.controller_specs_ = node["controller-specs"].as<std::vector <iai_gazebo_controllers::ControllerSpec> >();
      rhs.sim_start_delay_ = node["sim-start-delay"].as<double>();
      rhs.move_start_delay_ = node["move-start-delay"].as<double>();
      rhs.log_delay_ = node["log-delay"].as<double>();
      rhs.controlled_model_ = node["controlled-model"].as<std::string>();
      rhs.observed_model_ = node["observed-model"].as<std::string>();

      return true; 
    } 
  };

} 

#endif // IAI_GAZEBO_CONTROLLERS_YAML_PARSING_HH 

#ifndef IAI_GAZEBO_CONTROLLERS_YAML_PARSING_HH 
#define IAI_GAZEBO_CONTROLLERS_YAML_PARSING_HH 

#include <yaml-cpp/yaml.h>
#include <iai_gazebo_controllers/experiment_spec.hh>

namespace YAML
{  
  inline bool is_controller_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["controller-file"] &&
        node["controller-file"].IsScalar() && node["max-cmd-buffer-size"] &&
        node["max-cmd-buffer-size"].IsScalar();
  }

  inline bool is_experiment_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 3) && node["sim-start-delay"] &&
        node["sim-start-delay"].IsScalar() && node["move-start-delay"] && 
        node["move-start-delay"].IsScalar() && node["controller-specs"] &&
        node["controller-specs"].IsSequence();
  }

  template<> 
  struct convert<iai_gazebo_controllers::ControllerSpec>  
  { 
     
    static Node encode(const iai_gazebo_controllers::ControllerSpec& rhs)
    { 
      Node node; 
      node["controller-file"] = rhs.controller_file_;
      node["max-cmd-buffer-size"] = rhs.max_cmd_buffer_size_;
      return node; 
    } 
   
    static bool decode(const Node& node, iai_gazebo_controllers::ControllerSpec& rhs)
    { 
      if(!is_controller_spec(node)) 
        return false; 
   
      rhs.controller_file_ = node["controller-file"].as<std::string>();
      rhs.max_cmd_buffer_size_ = node["max-cmd-buffer-size"].as<size_t>();
 
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

      return node; 
    } 
   
    static bool decode(const Node& node, iai_gazebo_controllers::ExperimentSpec& rhs)
    { 
      if(!is_experiment_spec(node)) 
        return false; 
   
      rhs.controller_specs_ = node["controller-specs"].as<std::vector <iai_gazebo_controllers::ControllerSpec> >();
      rhs.sim_start_delay_ = node["sim-start-delay"].as<double>();
      rhs.move_start_delay_ = node["move-start-delay"].as<double>();
 
      return true; 
    } 
  };

} 

#endif // IAI_GAZEBO_CONTROLLERS_YAML_PARSING_HH 

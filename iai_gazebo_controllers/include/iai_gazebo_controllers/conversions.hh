#ifndef IAI_GAZEBO_CONTROLLERS_CONVERSIONS_HH 
#define IAI_GAZEBO_CONTROLLERS_CONVERSIONS_HH 

#include <fstream>
#include <iostream>
#include <fccl_conversions/YamlParser.h>
#include <yaml-cpp/yaml.h>
#include <iai_gazebo_controllers/motion_description.hh>

namespace iai_gazebo_controllers
{  
  void operator>> (const YAML::Node& node, MotionDescription& m)
  {
    node["name"] >> m.name_;
    node["finish-delay"] >> m.finish_delay_;
    assert(m.finish_delay_ >= 0.0);
    using fccl::conversions::operator>>;
    node["constraints"] >> m.constraints_;
  }

  std::ostream& operator<<(std::ostream& os, const MotionDescription& m)
  {
    using fccl::base::operator<<;

    os << "MotionDescription:\n";
    os << "name: " << m.name_ << "\n";
    os << "finish-delay: " << m.finish_delay_ << "\n";
    os << "constraints: " << m.constraints_;

    return os;
  }
} 

#endif // IAI_GAZEBO_CONTROLLERS_CONVERSIONS_HH 

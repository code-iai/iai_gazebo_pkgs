#ifndef IAI_GAZEBO_CONTROLLERS_MOTION_DESCRIPTION_HH 
#define IAI_GAZEBO_CONTROLLERS_MOTION_DESCRIPTION_HH 

#include <fccl/base/ConstraintArray.h>
#include <string>

namespace iai_gazebo_controllers
{
  class MotionDescription 
  { 
    public:
      std::string name_;
      double finish_delay_;
      fccl::base::ConstraintArray constraints_;
  };
}

#endif //IAI_GAZEBO_CONTROLLERS_MOTION_DESCRIPTION_HH 

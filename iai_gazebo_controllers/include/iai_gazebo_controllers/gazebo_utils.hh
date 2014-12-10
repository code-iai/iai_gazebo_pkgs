#ifndef IAI_GAZEBO_CONTROLLERS_GAZEBO_UTILS_HH 
#define IAI_GAZEBO_CONTROLLERS_GAZEBO_UTILS_HH 

#include <gazebo/gazebo.hh>
#include <kdl/frames.hpp>

namespace iai_gazebo_controllers
{
  /**
   * Looks up the value of an sdf element.
   * @param key the key used for the look-up.
   * @param sdf the actual sdf element.
   * @param value return value, rebound if key present.
   * @returns whether the key is present in the sdf element.
   */
  template<typename T>
  bool GetSDFValue(const std::string& key, sdf::ElementPtr sdf, T& value)
  {
    if (sdf->HasElement(key))
    {
      value = sdf->Get<T>(key);
      return true;
    }
    else
      return false;
  }

  /**
   * Looks up the value of an sdf element.
   * @param key the key used for the look-up.
   * @param sdf the actual sdf element.
   * @param value return value; rebound to value if key present, else rebound to default.
   * @param default default value in case key is not present.
   * @returns whether the key is present in the sdf element.
   */
  template<typename T>
  bool GetSDFValue(const std::string& key, sdf::ElementPtr sdf, T& value, 
      const T& default_value)
  {
    if (sdf->HasElement(key))
    {
      value = sdf->Get<T>(key);
      return true;
    }
    else
    {
      value = default_value;
      return false;
    }
  }

  class Twist
  {
    public:
      Twist() {}
      ~Twist() {}
      Twist(const gazebo::math::Vector3& linear_velocity,
          const gazebo::math::Vector3& angular_velocity) :
        linear_velocity_(linear_velocity), angular_velocity_(angular_velocity) {}

      gazebo::math::Vector3 linear_velocity_, angular_velocity_;
  };

  KDL::Frame toKDL(const gazebo::math::Pose& pose)
  {
    KDL::Rotation rot = KDL::Rotation::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    KDL::Vector pos = KDL::Vector(pose.pos.x, pose.pos.y, pose.pos.z); 
    return KDL::Frame(rot, pos);
  }

  gazebo::math::Vector3 toGazebo(const KDL::Vector& v)
  {
    return gazebo::math::Vector3(v.x(), v.y(), v.z());
  }

  Twist toGazebo(const KDL::Twist& twist)
  {
    return Twist(toGazebo(twist.vel), toGazebo(twist.rot));
  }
}
#endif //IAI_GAZEBO_CONTROLLERS_GAZEBO_UTILS_HH 

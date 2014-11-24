#ifndef IAI_GAZEBO_CONTROLLERS_GAZEBO_UTILS_HH 
#define IAI_GAZEBO_CONTROLLERS_GAZEBO_UTILS_HH 

#include <gazebo/gazebo.hh>

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
      value = sdf->Get<std::string>(key);
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
      value = sdf->Get<std::string>(key);
      return true;
    }
    else
    {
      value = default_value;
      return false;
    }
  }

  void VelocityControlLink(const gazebo::math::Vector3& linear_velocity,
      const gazebo::math::Vector3& angular_velocity,
      gazebo::physics::LinkPtr link)
  {
    link->SetLinearVel(linear_velocity);
    link->SetAngularVel(angular_velocity);
  }
}
#endif //IAI_GAZEBO_CONTROLLERS_GAZEBO_UTILS_HH 


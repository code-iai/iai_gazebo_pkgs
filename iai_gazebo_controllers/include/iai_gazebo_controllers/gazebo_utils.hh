#ifndef IAI_GAZEBO_CONTROLLERS_GAZEBO_UTILS_HH 
#define IAI_GAZEBO_CONTROLLERS_GAZEBO_UTILS_HH 

#include <gazebo/gazebo.hh>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

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

  inline KDL::Rotation toKDL(const gazebo::math::Quaternion& rot)
  {
    return KDL::Rotation::Quaternion(rot.x, rot.y, rot.z, rot.w);
  }

  inline KDL::Vector toKDL(const gazebo::math::Vector3& pos)
  {
    return KDL::Vector(pos.x, pos.y, pos.z); 
  }

  inline KDL::Frame toKDL(const gazebo::math::Pose& pose)
  {
    return KDL::Frame(toKDL(pose.rot), toKDL(pose.pos));
  }

  inline gazebo::math::Vector3 toGazebo(const KDL::Vector& v)
  {
    return gazebo::math::Vector3(v.x(), v.y(), v.z());
  }

  inline gazebo::math::Quaternion toGazebo(const KDL::Rotation& rot)
  {
    double x, y, z, w;
    rot.GetQuaternion(x, y, z, w);
    return gazebo::math::Quaternion(w, x, y, z);
  }


  inline Twist toGazebo(const KDL::Twist& twist)
  {
    return Twist(toGazebo(twist.vel), toGazebo(twist.rot));
  }

  inline Eigen::VectorXd PoseToGiskardInputs(const gazebo::math::Pose& pose) 
  {
    Eigen::VectorXd result(6);

    result(0) = pose.pos.x;
    result(1) = pose.pos.y;
    result(2) = pose.pos.z;

    toKDL(pose.rot).GetEulerZYX(result(3), result(4), result(5));

    return result;
  }

  inline std::vector<double> toSTL(const Eigen::VectorXd& v)
  {
    std::vector<double> result;
    for(size_t i=0; i<v.rows(); ++i)
      result.push_back(v(i));

    return result;
  }
}
#endif //IAI_GAZEBO_CONTROLLERS_GAZEBO_UTILS_HH 

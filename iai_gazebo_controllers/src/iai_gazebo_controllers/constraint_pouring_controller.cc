#include <iai_gazebo_controllers/constraint_pouring_controller.hh>
#include <iai_gazebo_controllers/gazebo_utils.hh>
#include <boost/bind.hpp>
#include <fstream>
#include <iostream>
#include <fccl_conversions/YamlParser.h>
#include <yaml-cpp/yaml.h>

using namespace gazebo;
using namespace fccl::base;
using namespace fccl::utils;
using namespace fccl::kdl;

namespace iai_gazebo_controllers
{  
  #define DELTA_DERIVATIVE 0.01
  #define DEFAULT_CYCLE_TIME 0.001

  void operator>> (const YAML::Node& node, MotionDescription& m)
  {
    node["name"] >> m.name_;
    node["start-delay"] >> m.start_delay_;
    using fccl::conversions::operator>>;
    node["constraints"] >> m.constraints_;
  }

  std::ostream& operator<<(std::ostream& os, const MotionDescription& m)
  {
    using fccl::base::operator<<;

    os << "MotionDescription:\n";
    os << "name: " << m.name_ << "\n";
    os << "start-delay: " << m.start_delay_ << "\n";
    os << "constraints: " << m.constraints_;

    return os;
  }
 
  void ConstraintPouringController::Load(physics::ModelPtr self, 
      sdf::ElementPtr self_description)
  {
    printf("Loading ConstraintPouringController...\n");
    this->self_description_ = self_description;
    this->self_ = self;

    ReadMotionDescriptions();
    InitController();
    SetupConnections();
  }

  void ConstraintPouringController::UpdateCallback(const common::UpdateInfo& info)
  {
   FillTransformMap();
    controller_.update(transforms_, DELTA_DERIVATIVE, getCycleTime(DEFAULT_CYCLE_TIME).Double());
    PerformVelocityControl(toGazebo(controller_.desiredTwist().numerics()));
    last_control_time_ = getCurrentSimTime();
  }

  void ConstraintPouringController::InitController()
  {
    self_->SetGravityMode(false);

    last_control_time_ = getCurrentSimTime();

    controller_.init(motions_[0].constraints_);
    fccl::control::PIDGains gains;
    gains.init(controller_.constraints().names());
    gains.p().numerics()(0) = 50.0;
    controller_.setGains(gains); 
    FillTransformMap();
    controller_.start(transforms_, getCycleTime(DEFAULT_CYCLE_TIME).Double());
  }

  void ConstraintPouringController::ReadMotionDescriptions()
  {
    std::string motion_file = "motions/sample-motion1.yaml";

    std::ifstream file_in(motion_file.c_str());
    assert(file_in.good());

    motions_.clear();
    YAML::Parser parser(file_in);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    MotionDescription tmp_motion;
    for(unsigned int i=0; i<doc.size(); ++i)
    {
      doc[i] >> tmp_motion;
      motions_.push_back(tmp_motion);
      std::cout << tmp_motion << "\n\n";

      assert(tmp_motion.constraints_.isValid());
    }
  }

  void ConstraintPouringController::SetupConnections()
  {
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ConstraintPouringController::UpdateCallback, this, _1));
  }

  void ConstraintPouringController::PerformVelocityControl(const Twist& twist)
  {
    gazebo::physics::LinkPtr link = self_->GetLinks()[0];
    link->SetLinearVel(twist.linear_velocity_);
    link->SetAngularVel(twist.angular_velocity_);
  }

  void ConstraintPouringController::FillTransformMap()
  {
    physics::ModelPtr stove = self_->GetWorld()->GetModel("PancakeMaker");
    math::Pose stove_pose = stove->GetLinks()[0]->GetWorldPose();
    math::Pose cup_pose = self_->GetLinks()[0]->GetWorldPose();
 
    Transform cup_transform, stove_transform; 
    cup_transform.semantics().reference().setName("World");
    cup_transform.semantics().target().setName("Cup");
    stove_transform.semantics().reference().setName("World");
    stove_transform.semantics().target().setName("PancakeMaker");
    cup_transform.numerics() = toKDL(cup_pose);
    stove_transform.numerics() = toKDL(stove_pose);
    transforms_.setTransform(cup_transform);
    transforms_.setTransform(stove_transform);
  }
 
  gazebo::common::Time ConstraintPouringController::getCurrentSimTime() const
  {
    return self_->GetWorld()->GetSimTime();
  }

  gazebo::common::Time ConstraintPouringController::getCycleTime(double default_cycle_time) const
  {
    common::Time cycle_time = getCurrentSimTime() - last_control_time_;
    if(cycle_time.Double() <= 0.0)
    {
      printf("WARNING: resetting negative cycle time to default: %f.\n", default_cycle_time);
      printf("Now: %f, last: %f, cycle: %f\n", getCurrentSimTime().Double(),
          last_control_time_.Double(), cycle_time.Double());
      cycle_time = common::Time(default_cycle_time);
    }
    return cycle_time;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ConstraintPouringController)
}

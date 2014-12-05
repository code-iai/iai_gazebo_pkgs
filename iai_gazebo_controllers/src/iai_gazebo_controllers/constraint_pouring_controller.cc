#include <iai_gazebo_controllers/constraint_pouring_controller.hh>
#include <iai_gazebo_controllers/gazebo_utils.hh>
#include <boost/bind.hpp>

using namespace gazebo;
using namespace fccl::base;
using namespace fccl::utils;
using namespace fccl::kdl;

namespace iai_gazebo_controllers
{  
  void ConstraintPouringController::Load(physics::ModelPtr self, 
      sdf::ElementPtr self_description)
  {
    printf("Loading ConstraintPouringController...\n");
    this->self_description_ = self_description;
    this->self_ = self;

    ReadPluginParameters();
    InitController();
    SetupConnections();
  }

  void ConstraintPouringController::UpdateCallback(const common::UpdateInfo& info)
  {
    FillTransformMap();
    //TODO: add time calcuation and pass it to update()
    controller_.update(transforms_);
    PerformVelocityControl(toGazebo(controller_.desiredTwist().numerics()));
    std::cout << "desired twist: " << controller_.desiredTwist() << "\n";
  }

  void ConstraintPouringController::InitController()
  {
    self_->SetGravityMode(false);

    controller_.init(GetConstraints());
    fccl::control::PIDGains gains;
    gains.init(controller_.constraints().names());
    gains.p().numerics()(0) = 10.0;
    controller_.setGains(gains); 
    FillTransformMap();
    controller_.start(transforms_);
  }

  void ConstraintPouringController::ReadPluginParameters()
  {
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
 
  fccl::base::ConstraintArray ConstraintPouringController::GetConstraints() 
  {
    // init features
    Feature stove_top_plane, cup_main_axis;
    cup_main_axis.semantics().type() = LINE_FEATURE;
    cup_main_axis.semantics().reference().setName("Cup");
    cup_main_axis.semantics().name().setName("main axis of cup");
    cup_main_axis.orientation() = KDL::Vector(0.0, 0.0, 1.0);

    stove_top_plane.semantics().type() = PLANE_FEATURE;
    stove_top_plane.semantics().reference().setName("PancakeMaker");
    stove_top_plane.semantics().name().setName("center of top plane of pancake stove");
    stove_top_plane.position() = KDL::Vector(0.0, 0.0, 0.05);
    stove_top_plane.orientation() = KDL::Vector(0.0, 0.0, 1.0);

    // assembling constraints
    Constraint above_constraint;    
    above_constraint.toolFeature() = cup_main_axis;
    above_constraint.objectFeature() = stove_top_plane;
    above_constraint.semantics().reference().setName("World");
    above_constraint.semantics().name().setName("Cup above stove constraint.");
    above_constraint.semantics().type().setName("above");
    above_constraint.lowerBoundary() = 0.3;
    above_constraint.upperBoundary() = 0.4;
    above_constraint.maxVelocity() = 0.1;
    above_constraint.maxAcceleration() = 0.2;
    above_constraint.maxJerk() = 0.4;

    printf("Constraint valid: %d\n", above_constraint.isValid());
    std::vector<Constraint> constraint_vector;
    constraint_vector.push_back(above_constraint);
    ConstraintArray constraints;
    constraints.init(constraint_vector);

    return constraints;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ConstraintPouringController)
}

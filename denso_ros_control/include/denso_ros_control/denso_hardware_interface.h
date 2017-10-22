#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <iterator>
#include "denso_robot.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSaturationHandle;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSaturationInterface;
using joint_limits_interface::PositionJointSoftLimitsInterface;

using namespace transmission_interface;
class DensoRobotHW : public hardware_interface::RobotHW
{
public:
  DensoRobotHW() : raddeg_trans_(180/M_PI,0), m_mm_trans_(100, 0) // for gripper
  { 
    robot_ = boost::shared_ptr<DensoRobot>(new DensoRobot());
  }
  virtual ~DensoRobotHW() {
    robot_.reset();
  }

  void appendJoint(const KDL::SegmentMap::const_iterator segment, std::vector<const KDL::Joint*>& joints) {
    joints.push_back(&(segment->second.segment.getJoint()));
    ROS_DEBUG_STREAM("append joint: " << segment->second.segment.getJoint().getName() << " at " << segment->first << ", childrensize: " << segment->second.children.size());
    for (int i = 0; i < segment->second.children.size(); i++) {
      appendJoint(segment->second.children[i], joints);
    }
  }

  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) {
    std::string robot_description;
    if (!robot_hw_nh.getParam("robot_description", robot_description)) {
      ROS_ERROR("Failed to get robot_description");
      return false;
    }
    urdf::Model model;
    if (!model.initString(robot_description)){
      ROS_ERROR("Failed to parse robot_description");
      return false;
    }
    KDL::Tree tree;
    //if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    if (!kdl_parser::treeFromString(robot_description, tree)){
      ROS_ERROR("Failed to create tree from urdf");
      return false;
    }
    KDL::SegmentMap::const_iterator root_segment = tree.getRootSegment();
    std::vector<const KDL::Joint*> joints;
    appendJoint(root_segment->second.children[0], joints);
    // skip fixed joint https://stackoverflow.com/questions/4478636/stdremove-if-lambda-not-removing-anything-from-the-collection
    auto new_end = std::remove_if(joints.begin(), joints.end(),
                                  [](const KDL::Joint*& joint)->bool
                                  { return joint->getType() == KDL::Joint::None; });
    joints.erase(new_end, joints.end());

    int num_joints = joints.size();
    cmd_.resize(num_joints);
    pos_.resize(num_joints);
    vel_.resize(num_joints);
    eff_.resize(num_joints);
    a_pos_.resize(num_joints);
    a_pos_prev_.resize(num_joints);
    a_vel_.resize(num_joints);
    a_eff_.resize(num_joints);
    a_cmd_.resize(num_joints);

    a_state_data_.resize(num_joints);
    a_cmd_data_.resize(num_joints);
    j_state_data_.resize(num_joints);
    j_cmd_data_.resize(num_joints);

    // connect and register the joint state interface

    for (int i = 0; i < joints.size(); i++) {
      hardware_interface::JointStateHandle state_handle(joints[i]->getName(), &pos_[i], &vel_[i], &eff_[i]);
      jnt_state_interface_.registerHandle(state_handle);

      j_state_data_[i].position.push_back(&pos_[i]);
      j_state_data_[i].velocity.push_back(&vel_[i]);
      j_state_data_[i].effort.push_back(&eff_[i]);
      a_state_data_[i].position.push_back(&a_pos_[i]);
      a_state_data_[i].velocity.push_back(&a_vel_[i]);
      a_state_data_[i].effort.push_back(&a_eff_[i]);
    }
    registerInterface(&jnt_state_interface_);

    // connect and register the joint position interface
    for (int i = 0; i < joints.size(); i++) {
      std::string jointname = joints[i]->getName();
      hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(jointname), &cmd_[i]);
      jnt_eff_interface_.registerHandle(pos_handle);

      a_cmd_data_[i].position.push_back(&a_cmd_[i]); // Velocity and effort vectors are unused
      j_cmd_data_[i].position.push_back(&cmd_[i]); // Velocity and effort vectors are unused

      // Register transmissions to each interface
      act_to_jnt_state_.registerHandle(ActuatorToJointStateHandle(jointname + "_raddeg_trans_",
            &raddeg_trans_,
            a_state_data_[i],
            j_state_data_[i]));

      jnt_to_act_pos_.registerHandle(JointToActuatorPositionHandle(jointname + "_raddeg_trans_",
            &raddeg_trans_,
            a_cmd_data_[i],
            j_cmd_data_[i]));

      // Register joint limits
      JointLimits limits;
      SoftJointLimits soft_limits;
      const bool urdf_limits_ok = getJointLimits(model.getJoint(jointname), limits);
      if (!urdf_limits_ok) {
        ROS_WARN("urdf limits int joint %s is not defined", jointname.c_str());
      }
      const bool urdf_soft_limits_ok = getSoftJointLimits(model.getJoint(jointname), soft_limits);
      if (!urdf_soft_limits_ok) {
        ROS_WARN("urdf soft limits int joint %s is not defined", jointname.c_str());
      }
      // Register handle in joint limits interface
      PositionJointSoftLimitsHandle limits_handle(pos_handle, // We read the state and read/write the command
          limits,       // Limits spec
          soft_limits);  // Soft limits spec
      //PositionJointSaturationHandle limits_handle(pos_handle, // We read the state and read/write the command
      //    limits);       // Limits spec

      jnt_limits_interface_.registerHandle(limits_handle);
    }
    registerInterface(&jnt_eff_interface_);

    if (!robot_->initialize(root_nh, num_joints)) {
      ROS_ERROR("failed to initialize denso robot");
      robot_.reset();
      return false;
    }
    if(!robot_->read(a_pos_)) {
      ROS_WARN("failed to read joint values from robot");
      robot_.reset();
      return false;
    }

    std::copy(a_pos_.begin(), a_pos_.end(), a_pos_prev_.begin());
    act_to_jnt_state_.propagate();

    return true;
  }

  void read(const ros::Time& time, const ros::Duration& period) {
    // do nothing

  }

  void write(const ros::Time& time, const ros::Duration& period) {
    jnt_limits_interface_.enforceLimits(period);
    jnt_to_act_pos_.propagate();

    ROS_DEBUG_STREAM("a_cmd_: " << a_cmd_);
    ROS_DEBUG_STREAM("cmd_: " << cmd_);
    if(!robot_->write(a_cmd_, a_pos_)) {
      ROS_ERROR("failed to move robot!");
    }
    for (int i = 0; i < a_pos_.size(); i++) {
      a_vel_[i] = (a_pos_[i] - a_pos_prev_[i])/period.toSec();
    }
    act_to_jnt_state_.propagate();
    std::copy(a_pos_.begin(), a_pos_.end(), a_pos_prev_.begin());
  }

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.008); }
private:
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_eff_interface_;
  ActuatorToJointStateInterface    act_to_jnt_state_;
  JointToActuatorPositionInterface jnt_to_act_pos_;
  SimpleTransmission       raddeg_trans_;
  SimpleTransmission       m_mm_trans_;
  // Actuator and joint space variables: wrappers around raw data
  std::vector<ActuatorData> a_state_data_;
  std::vector<ActuatorData> a_cmd_data_;
  std::vector<JointData> j_state_data_;
  std::vector<JointData> j_cmd_data_;
  //PositionJointSaturationInterface jnt_limits_interface_;
  PositionJointSoftLimitsInterface jnt_limits_interface_;

  // joints
  std::vector<double> cmd_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;

  // actuators
  std::vector<double> a_pos_, a_pos_prev_;
  std::vector<double> a_vel_;
  std::vector<double> a_eff_;
  std::vector<double> a_cmd_;

  boost::shared_ptr<DensoRobot> robot_;
};


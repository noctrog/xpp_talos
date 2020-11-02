#include <ros/ros.h>

#include <xpp_talos/inverse_kinematics_talos.h> // TODO
#include <xpp_msgs/topic_names.h>

#include <xpp_vis/urdf_visualizer.h>
#include <xpp_vis/cartesian_joint_converter.h>

#include <xpp_states/endeffector_mappings.h>

using namespace xpp;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "talos_urdf_visualizer");

  const std::string joint_desired_talos = "xpp/joint_talos_des";

  // auto ik = std::make_shared<InverseKinematicsHyq2>();
  // CartesianJointConverter inv_kin_converter(ik,
						// xpp_msgs::robot_state_desired,
						// joint_desired_talos);

  // int n_ee = ik->GetEECount();
  // int n_j  = HyqlegJointCount;
  // std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  // joint_names.at(LL1) = "leg_left_1_joint";
  // joint_names.at(LL2) = "leg_left_2_joint";
  // joint_names.at(LL3) = "leg_left_3_joint";
  // joint_names.at(LL4) = "leg_left_4_joint";
  // joint_names.at(LL5) = "leg_left_5_joint";
  // joint_names.at(LL6) = "leg_left_6_joint";
  // joint_names.at(LR1) = "leg_right_1_joint";
  // joint_names.at(LR2) = "leg_right_2_joint";
  // joint_names.at(LR3) = "leg_right_3_joint";
  // joint_names.at(LR4) = "leg_right_4_joint";
  // joint_names.at(LR5) = "leg_right_5_joint";
  // joint_names.at(LR6) = "leg_right_6_joint";
  // joint_names.at(T1)  = "torso_1_joint";
  // joint_names.at(T2)  = "torso_2_joint";
  // joint_names.at(AL1) = "arm_left_1_joint";
  // joint_names.at(AL2) = "arm_left_2_joint";
  // joint_names.at(AL3) = "arm_left_3_joint";
  // joint_names.at(AL4) = "arm_left_4_joint";
  // joint_names.at(AL5) = "arm_left_5_joint";
  // joint_names.at(AL6) = "arm_left_6_joint";
  // joint_names.at(AL7) = "arm_left_7_joint";
  // joint_names.at(GL)  = "gripper_left_joint";
  // joint_names.at(AR1) = "arm_right_1_joint";
  // joint_names.at(AR2) = "arm_right_2_joint";
  // joint_names.at(AR3) = "arm_right_3_joint";
  // joint_names.at(AR4) = "arm_right_4_joint";
  // joint_names.at(AR5) = "arm_right_5_joint";
  // joint_names.at(AR6) = "arm_right_6_joint";
  // joint_names.at(AR7) = "arm_right_7_joint";
  // joint_names.at(GR)  = "gripper_right_joint";
  // joint_names.at(H1)  = "head_1_joint";
  // joint_names.at(H2)  = "head_2_joint";

  // std::string urdf = "talos_rviz_urdf_robot_description";
  // UrdfVisualizer node(urdf, joint_names, "base", "world",
			  // joint_desired_talos, "talos");

  ::ros::spin();

  return 1;
}


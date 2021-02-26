#include <ros/ros.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_talos/inverse_kinematics_talos.h>

#include <xpp_talos/talos_cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

#include <xpp_states/endeffector_mappings.h>

using namespace xpp;

int main(int argc, char *argv[]) {
  ::ros::init(argc, argv, "talos_urdf_visualizer");
  const std::string joint_desired_talos = "xpp/joint_talos_des";

  auto ik = std::make_shared<InverseKinematicsTalos>();
  TalosCartesianJointConverter inv_kin_converter(ik,
						 xpp_msgs::robot_state_desired,
						 joint_desired_talos);

  std::vector<UrdfVisualizer::URDFName> joint_names(14);
  joint_names.at(AL1) = "arm_left_1_joint";
  joint_names.at(AL2) = "arm_left_2_joint";
  joint_names.at(AL3) = "arm_left_3_joint";
  joint_names.at(AL4) = "arm_left_4_joint";
  joint_names.at(AL5) = "arm_left_5_joint";
  joint_names.at(AL6) = "arm_left_6_joint";
  joint_names.at(AL7) = "arm_left_7_joint";
  joint_names.at(AR1) = "arm_right_1_joint";
  joint_names.at(AR2) = "arm_right_2_joint";
  joint_names.at(AR3) = "arm_right_3_joint";
  joint_names.at(AR4) = "arm_right_4_joint";
  joint_names.at(AR5) = "arm_right_5_joint";
  joint_names.at(AR6) = "arm_right_6_joint";
  joint_names.at(AR7) = "arm_right_7_joint";

  std::string urdf = "talos_rviz_urdf_robot_description";
  UrdfVisualizer node(urdf, joint_names, "base_link", "world",
		      joint_desired_talos, "talos");

  ::ros::spin();

  return 1;
}

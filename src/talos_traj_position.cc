#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_states/convert.h>

#include <xpp_talos/inverse_kinematics_talos.h>

/**
 * Takes a RobotStateCartesian and calculates the inverse kinematics
 * for the Talos robot
 */
Eigen::VectorXd GetJointAngles(const xpp_msgs::RobotStateCartesian::ConstPtr i,
			       const xpp::InverseKinematicsTalos &ik) {
  auto cart = xpp::Convert::ToXpp(*i);

  // transform feet from world -> base frame
  Eigen::Matrix3d B_R_W =
      cart.base_.ang.q.normalized().toRotationMatrix().inverse();
  xpp::EndeffectorsPos ee_B(cart.ee_motion_.GetEECount());
  xpp::EndeffectorsRot ee_R(cart.ee_motion_.GetEECount());
  for (auto ee : ee_B.GetEEsOrdered()) {
    ee_B.at(ee) = B_R_W * (cart.ee_motion_.at(ee).p_ - cart.base_.lin.p_);
    ee_R.at(ee) = B_R_W;
  }

  Eigen::VectorXd q = ik.GetAllJointAngles(ee_B, ee_R).ToVec();

  return q;
}

/**
 * Takes a ROS bag of optimization results, and generates a
 * trajectory_msgs/JointTrajectory
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rosbag_trajectory_player");
  ros::NodeHandle nh;

  auto pub_l = nh.advertise<trajectory_msgs::JointTrajectory>("/left_leg_controller/command", 100);
  auto pub_r = nh.advertise<trajectory_msgs::JointTrajectory>("/right_leg_controller/command", 100);

  // Get bag file, if not specified, use towr default bag
  std::string name;
  if (argc == 1) {
    struct passwd *pw = getpwuid(getuid());
    std::string home_dir(pw->pw_dir);
    name = home_dir + "/.ros/towr_trajectory";
  } else {
    name = argv[1];
  }

  // Open bag
  rosbag::Bag bag_r;
  bag_r.open(name + ".bag", rosbag::bagmode::Read);
  ROS_INFO_STREAM("Reading from bag " + bag_r.getFileName());

  // Topic to load
  std::string robot_state_topic("/xpp/state_des");
  std::vector<std::string> topics;
  topics.push_back(robot_state_topic);

  rosbag::View view(bag_r, rosbag::TopicQuery(topics));

  // Prepare trajectory message
  trajectory_msgs::JointTrajectory traj_left, traj_right;
  traj_left.header.frame_id = "";
  traj_left.header.stamp = ros::Time::now();
  traj_right.header = traj_left.header;
  traj_left.joint_names.emplace_back("leg_left_1_joint");
  traj_left.joint_names.emplace_back("leg_left_2_joint");
  traj_left.joint_names.emplace_back("leg_left_3_joint");
  traj_left.joint_names.emplace_back("leg_left_4_joint");
  traj_left.joint_names.emplace_back("leg_left_5_joint");
  traj_left.joint_names.emplace_back("leg_left_6_joint");
  traj_right.joint_names.emplace_back("leg_right_1_joint");
  traj_right.joint_names.emplace_back("leg_right_2_joint");
  traj_right.joint_names.emplace_back("leg_right_3_joint");
  traj_right.joint_names.emplace_back("leg_right_4_joint");
  traj_right.joint_names.emplace_back("leg_right_5_joint");
  traj_right.joint_names.emplace_back("leg_right_6_joint");
  // traj_left.points.resize(6);
  // traj_right.points.resize(6);

  // TODO automatically get frecuency
  ROS_INFO_STREAM("Calculating joint positions...");
  xpp::InverseKinematicsTalos ik;
  size_t current_t = 0;
  for (rosbag::MessageInstance const m : view) {
    xpp_msgs::RobotStateCartesian::ConstPtr i =
	m.instantiate<xpp_msgs::RobotStateCartesian>();

    if (i) {
      // Perform inverse kinematics
      Eigen::VectorXd q = GetJointAngles(i, ik);

      // Calculate corresponding time for position
      double time = current_t * 0.01;
      traj_left.points.emplace_back();
      traj_right.points.emplace_back();
      traj_left.points.back().time_from_start = ros::Duration(time);
      traj_right.points.back().time_from_start = ros::Duration(time);
      // Add values to JointTrajectory
      for (int i = 0; i < 6; ++i) {
	// Positions
	traj_left.points.back().positions.push_back(q[i]);
      }
      for (int i = 6; i < q.size(); ++i) {
	traj_right.points.back().positions.push_back(q[i]);
      }
    }

    current_t++;
  }

  // Publicar ambos mensajes
  ROS_INFO_STREAM("Publishing messages");

  // Wait for publishers
  ros::Duration(0.2).sleep();

  pub_l.publish(traj_left);
  pub_r.publish(traj_right);

  ros::spinOnce();

  ROS_INFO_STREAM("Done.");
  return 0;
}

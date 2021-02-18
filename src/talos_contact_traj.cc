#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_states/convert.h>

#include <xpp_talos/inverse_kinematics_talos.h>

#include <talos_wbc_controller/JointContactTrajectory.h>
#include <talos_wbc_controller/JointContactTrajectoryContacts.h>

typedef typename std::vector<bool> CurrentContacts;

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
 * Takes a RobotStateCartesian and retrieves an std::vector<bool>
 * representing the current contacts of the end effectos.
 */
CurrentContacts GetCurrentContact(const xpp_msgs::RobotStateCartesian::ConstPtr i)
{
  const auto cart = xpp::Convert::ToXpp(*i);
  const auto n_ee = cart.ee_motion_.GetEECount();

  // One trajectory per end effector
  CurrentContacts contact_traj(n_ee);

  // Retrieve the current contacts
  for (size_t j = 0; j < n_ee; ++j) {
    contact_traj[j] = i->ee_contact[j];
  }

  return contact_traj;
}

/**
 * Takes a trajectory msg and assigns the corresponding joints for
 * both legs
 */
void PrepareTrajMsg(talos_wbc_controller::JointContactTrajectory &msg) {
  msg.header.frame_id = "";
  msg.header.stamp = ros::Time::now();

  // Ordinary trajectory names
  msg.trajectory.joint_names.emplace_back("leg_left_1_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_2_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_3_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_4_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_5_joint");
  msg.trajectory.joint_names.emplace_back("leg_left_6_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_1_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_2_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_3_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_4_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_5_joint");
  msg.trajectory.joint_names.emplace_back("leg_right_6_joint");

  // Contact names
  msg.contact_link_names.emplace_back("leg_left_6_link");
  msg.contact_link_names.emplace_back("leg_right_6_link");
}

/**
 * Takes a ROS bag of optimization results, and generates a
 * trajectory_msgs/JointTrajectory
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rosbag_trajectory_player");
  ros::NodeHandle nh;

  auto pub = nh.advertise<talos_wbc_controller::JointContactTrajectory>(
      "/talos_trajectory_wbc_controller/command", 100);

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

  // Prepare trajectory messages
  // Move Talos to initial pose
  talos_wbc_controller::JointContactTrajectory init_traj;
  PrepareTrajMsg(init_traj);
  // Perform movement
  talos_wbc_controller::JointContactTrajectory traj;
  PrepareTrajMsg(traj);

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
      traj.trajectory.points.emplace_back();
      traj.trajectory.points.back().time_from_start = ros::Duration(time);
      // Add values to JointTrajectory
      for (int i = 0; i < 12; ++i) {
	// Positions
	traj.trajectory.points.back().positions.push_back(q[i]);
      }

      // Add values to the contact sequence
      CurrentContacts curr_contacts = GetCurrentContact(i);
      talos_wbc_controller::JointContactTrajectoryContacts cont_msg;
      cont_msg.contacts.resize(2);
      for (int i = 0; i < 2; ++i)
	cont_msg.contacts.at(i) = curr_contacts.at(i);
      cont_msg.time_from_start = traj.trajectory.points.back().time_from_start;
      traj.contacts.push_back(cont_msg);

      // If this is the initial pose, save it in the initialization
      // trajectory
      if (current_t == 0) {
	// Talos initial pose, all zeros
	init_traj.trajectory.points.emplace_back();
	init_traj.trajectory.points.back().time_from_start = ros::Duration(0.1);
	init_traj.trajectory.points.back().positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					     0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// Trajectory initial pose
	init_traj.trajectory.points.emplace_back(traj.trajectory.points.back());
	init_traj.trajectory.points.back().time_from_start = ros::Duration(1.0);

	// In the first segment the robot is touching the ground with both feet
	init_traj.contacts.resize(1);  // Only one segment
	init_traj.contacts[0].contacts.resize(2); // Two feet per segment
	init_traj.contacts[0].contacts.at(0) = true;
	init_traj.contacts[0].contacts.at(1) = true;
	init_traj.contacts[0].time_from_start = init_traj.trajectory.points.back().time_from_start;
      }
    }

    current_t++;
  }

  // Publish trajectory msgs
  ROS_INFO_STREAM("Publishing messages");

  // Wait for publishers to be ready
  ros::Duration(0.2).sleep();

  // Initialize robot
  pub.publish(init_traj);
  ros::Duration(1.5).sleep();

  // Play optimized trajectory
  pub.publish(traj);

  ros::spinOnce();

  ROS_INFO_STREAM("Done.");
  return 0;
}

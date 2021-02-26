#include <ros/ros.h>
#include <ros/package.h>

#include <cmath>
#include <iostream>

#include <xpp_talos/inverse_kinematics_talos.h>
#include <xpp_states/endeffector_mappings.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

namespace xpp {

  InverseKinematicsTalos::InverseKinematicsTalos()
  {
    // Retrieve the corresponding filename
    std::string model_file_path =
      ros::package::getPath("xpp_talos") + std::string("/urdf/talos_arms.urdf");

    // Build the kinematic tree
    if (not kdl_parser::treeFromFile(model_file_path, kinematic_tree_)) {
      ROS_ERROR("Failed to construct the KDL Tree for the Talos robot");
      exit(-1);
    }

    // Retrieve the kinematic chains
    kinematic_tree_.getChain("base_link", "gripper_left_base_link", left_arm_chain_);
    kinematic_tree_.getChain("base_link", "gripper_right_base_link", right_arm_chain_);

    // Create the forward kinematics solvers
    left_arm_fk_pos_.reset(new KDL::ChainFkSolverPos_recursive(left_arm_chain_));
    right_arm_fk_pos_.reset(new KDL::ChainFkSolverPos_recursive(right_arm_chain_));

    // Create inverse kinematics solvers
    // Velocity IK
    left_arm_ik_vel_.reset(new KDL::ChainIkSolverVel_pinv(left_arm_chain_));
    right_arm_ik_vel_.reset(new KDL::ChainIkSolverVel_pinv(right_arm_chain_));
    // Position IK - Max 100 iterations, stop at an accuracy of 10e-6
    left_arm_ik_pos_.reset(new KDL::ChainIkSolverPos_NR(left_arm_chain_, *left_arm_fk_pos_,
							*left_arm_ik_vel_, 1000, 100)); 
    right_arm_ik_pos_.reset(new KDL::ChainIkSolverPos_NR(right_arm_chain_, *right_arm_fk_pos_,
							*right_arm_ik_vel_, 1000, 100)); 

    // Initialize the estimations of the arm positions
    left_arm_last_pos_ = KDL::JntArray(left_arm_chain_.getNrOfJoints());
    right_arm_last_pos_ = KDL::JntArray(right_arm_chain_.getNrOfJoints());
  }

  Joints 
  InverseKinematicsTalos::GetAllJointAngles(const EndeffectorsPos& pos_B) const
  {
    using namespace biped;
    std::vector<Eigen::VectorXd> q_vec;

    // make sure always exactly 2 elements
    auto x_biped_B = pos_B.ToImpl();
    x_biped_B.resize(2, x_biped_B.front());

    // Left arm inverse kinematics
    KDL::Frame left_ee_frame(KDL::Vector(x_biped_B.at(L).x(),
					 x_biped_B.at(L).y(),
					 x_biped_B.at(L).z()));
    KDL::JntArray left_q_result(left_arm_chain_.getNrOfJoints());

    if (not left_arm_ik_pos_->CartToJnt(left_arm_last_pos_, left_ee_frame, left_q_result)) {
      ROS_ERROR("Failed to compute the IK of the left arm!");
      exit(-1);
    } else {
      q_vec.push_back(left_q_result.data);
    }
    
    // Right arm inverse kinematics
    KDL::Frame right_ee_frame(KDL::Vector(x_biped_B.at(R).x(),
					  x_biped_B.at(R).y(),
					  x_biped_B.at(R).z()));
    KDL::JntArray right_q_result(right_arm_chain_.getNrOfJoints());

    if (not right_arm_ik_pos_->CartToJnt(right_arm_last_pos_, right_ee_frame, right_q_result)) {
      ROS_ERROR("Failed to compute the IK of the right arm!");
      exit(-1);
    } else {
      q_vec.push_back(right_q_result.data);
    }

    Joints a(q_vec);
    return Joints(q_vec);
  }

  Joints 
  InverseKinematicsTalos::GetAllJointAngles(const EndeffectorsPos& pos_B,
					    const EndeffectorsRot& rot_B) const
  {
    using namespace biped;
    std::vector<Eigen::VectorXd> q_vec;

    // make sure always exactly 2 elements
    auto x_biped_B = pos_B.ToImpl();
    x_biped_B.resize(2, x_biped_B.front());

    // TODO Take orientation into account
    return GetAllJointAngles(pos_B);

    // Joints a(q_vec);
    // return Joints(q_vec);
  }
} /* namespace xpp */

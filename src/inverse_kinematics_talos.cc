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
    // Position IK - Max 100 iterations, stop at an accuracy of 10e-5
    left_arm_ik_pos_.reset(new KDL::ChainIkSolverPos_NR(left_arm_chain_, *left_arm_fk_pos_,
							*left_arm_ik_vel_, 100, 1e-5)); 
    right_arm_ik_pos_.reset(new KDL::ChainIkSolverPos_NR(right_arm_chain_, *right_arm_fk_pos_,
							*right_arm_ik_vel_, 1000, 1e-5)); 

    // Initialize the estimations of the arm positions
    left_arm_last_pos_.reset(new KDL::JntArray(left_arm_chain_.getNrOfJoints()));
    right_arm_last_pos_.reset(new KDL::JntArray(right_arm_chain_.getNrOfJoints()));
    // Initialize initial guess to a known position
    left_arm_last_pos_->data(1) =  M_PI / 4.0;
    left_arm_last_pos_->data(3) = -M_PI / 2.0;
    right_arm_last_pos_->data(1) = -M_PI / 4.0;
    right_arm_last_pos_->data(3) = -M_PI / 2.0;
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
    KDL::Vector left_ee_vector(x_biped_B.at(L).x(), x_biped_B.at(L).y(), x_biped_B.at(L).z());
    KDL::Rotation left_ee_rotation(KDL::Vector( 0.0, 0.0, 1.0),
				   KDL::Vector( 0.0, 1.0, 0.0),
				   KDL::Vector(-1.0, 0.0, 0.0));
    KDL::Frame left_ee_frame(left_ee_rotation, left_ee_vector);

    KDL::JntArray left_q_result(left_arm_chain_.getNrOfJoints());

    if (not left_arm_ik_pos_->CartToJnt(*left_arm_last_pos_, left_ee_frame, left_q_result)) {
      ROS_ERROR("Failed to compute the IK of the left arm!");
      // q_vec.push_back(Eigen::VectorXd::Zero(left_arm_chain_.getNrOfJoints()));
      q_vec.push_back(left_q_result.data);
    } else {
      q_vec.push_back(left_q_result.data);
    }
    
    *left_arm_last_pos_ = left_q_result;

    // Right arm inverse kinematics
    KDL::Vector right_ee_vector(x_biped_B.at(R).x(), x_biped_B.at(R).y(), x_biped_B.at(R).z());
    KDL::Rotation right_ee_rotation(KDL::Vector( 0.0, 0.0, 1.0),
				   KDL::Vector( 0.0, 1.0, 0.0),
				   KDL::Vector(-1.0, 0.0, 0.0));
    KDL::Frame right_ee_frame(right_ee_rotation, right_ee_vector);
    KDL::JntArray right_q_result(right_arm_chain_.getNrOfJoints());

    if (not right_arm_ik_pos_->CartToJnt(*right_arm_last_pos_, right_ee_frame, right_q_result)) {
      ROS_ERROR("Failed to compute the IK of the right arm!");
      // q_vec.push_back(Eigen::VectorXd::Zero(right_arm_chain_.getNrOfJoints()));
      q_vec.push_back(right_q_result.data);
    } else {
      q_vec.push_back(right_q_result.data);
    }

    *right_arm_last_pos_ = right_q_result;

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

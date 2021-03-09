#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <xpp_talos/inverse_kinematics_talos.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <cmath>
#include <iostream>
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

    // Pinocchio model
    talos_model_.reset(new pinocchio::Model);
    pinocchio::urdf::buildModel(model_file_path, pinocchio::JointModelFreeFlyer(), *talos_model_);
    ROS_INFO("Pinocchio model loaded success, robot name: %s", talos_model_->name.c_str());

    // Initialize pinocchio model data
    talos_data_.reset(new pinocchio::Data(*talos_model_));

    left_hand_id = talos_model_->getFrameId("gripper_left_base_link");
    right_hand_id = talos_model_->getFrameId("gripper_right_base_link");
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

  Joints
  InverseKinematicsTalos::GetAllJointVelocities(const EndeffectorsVel &vel_B,
						const Joints &pos_j) const
  {
    using namespace biped;
    std::vector<Eigen::VectorXd> qd_vec;

    // Convert joint position to KDL representation
    KDL::JntArray left_q(pos_j.GetNumJointsPerEE()), right_q(pos_j.GetNumJointsPerEE());
    for (size_t i = 0; i < pos_j.GetNumJoints(); ++i) {
      if (i < pos_j.GetNumJointsPerEE())
	left_q.data(i) = pos_j.GetJoint(i);
      else
	right_q.data(i-pos_j.GetNumJointsPerEE()) = pos_j.GetJoint(i);
    }

    // make sure always exactly 2 elements
    auto x_biped_B = vel_B.ToImpl();
    x_biped_B.resize(2, x_biped_B.front());

    // Left arm inverse kinematics
    KDL::Vector left_ee_vector(x_biped_B.at(L).x(), x_biped_B.at(L).y(), x_biped_B.at(L).z());
    KDL::Twist left_ee_twist(left_ee_vector, KDL::Vector::Zero());

    KDL::JntArray left_qd_result(left_arm_chain_.getNrOfJoints());
    if (not left_arm_ik_vel_->CartToJnt(left_q, left_ee_twist, left_qd_result)) {
      ROS_ERROR("Failed to compute the IK vel of the left arm!");
      // q_vec.push_back(Eigen::VectorXd::Zero(left_arm_chain_.getNrOfJoints()));
      qd_vec.push_back(left_qd_result.data);
    } else {
      qd_vec.push_back(left_qd_result.data);
    }
    
    // Right arm inverse kinematics
    KDL::Vector right_ee_vector(x_biped_B.at(R).x(), x_biped_B.at(R).y(), x_biped_B.at(R).z());
    KDL::Twist right_ee_twist(right_ee_vector, KDL::Vector::Zero());

    KDL::JntArray right_qd_result(right_arm_chain_.getNrOfJoints());
    if (not right_arm_ik_vel_->CartToJnt(right_q, right_ee_twist, right_qd_result)) {
      ROS_ERROR("Failed to compute the IK vel of the right arm!");
      // q_vec.push_back(Eigen::VectorXd::Zero(right_arm_chain_.getNrOfJoints()));
      qd_vec.push_back(right_qd_result.data);
    } else {
      qd_vec.push_back(right_qd_result.data);
    }

    Joints a(qd_vec);
    return Joints(qd_vec);
  }

  Joints
  InverseKinematicsTalos::GetAllJointAccelerations(const EndeffectorsAcc &acc_B,
						   const Joints &pos_j,
						   const Joints &vel_j) const
  {
    // Retrieve the end effector accelerations (towr does not calculate orientations)
    Eigen::VectorXd left_hand_acc(6), right_hand_acc(6);
    left_hand_acc.setZero(); right_hand_acc.setZero();
    left_hand_acc.head(3) << acc_B.at(0);
    right_hand_acc.head(3) << acc_B.at(1);

    // Retrieve the current robot state (spatial position and velocity)
    Eigen::VectorXd q  = Eigen::VectorXd::Zero(7 + pos_j.GetNumJoints());
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(6 + vel_j.GetNumJoints());
    for (size_t i = 0; i < pos_j.GetNumJoints(); ++i) {
      q(7+i)  = pos_j.GetJoint(i);
      qd(6+i) = vel_j.GetJoint(i);
    }

    // Compute all the jacobians needed and slice
    Eigen::MatrixXd J_left = GetFrameJacobian(left_hand_id).block(0, 6, 6, 7);
    Eigen::MatrixXd J_right = GetFrameJacobian(right_hand_id).block(0, 12, 6, 7);

    // Compute the pseudo-inverse: https://www.researchgate.net/publication/260354295_Extended_Jacobian_For_Redundant_Robots_Obtained_From_The_Kinematics_Constraints
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(7, 7);
    Eigen::MatrixXd Jpinv_left = ComputePseudoInverse(J_left, W);
    Eigen::MatrixXd Jpinv_right = ComputePseudoInverse(J_right, W);

    // TODO: Compute Jd * qd
    // https://github.com/stack-of-tasks/pinocchio/issues/1395
    pinocchio::forwardKinematics(*talos_model_, *talos_data_, q, qd, 0*qd);
    pinocchio::computeJointJacobiansTimeVariation(*talos_model_, *talos_data_, q, qd);
    auto dJq_left_m = pinocchio::getFrameClassicalAcceleration(*talos_model_, *talos_data_,
							     left_hand_id,
							     pinocchio::ReferenceFrame::WORLD);
    auto dJq_right_m = pinocchio::getFrameClassicalAcceleration(*talos_model_, *talos_data_,
							     right_hand_id,
							     pinocchio::ReferenceFrame::WORLD);
    Eigen::VectorXd dJq_left(6), dJq_right(6); 
    dJq_left << dJq_left_m.linear(), dJq_left_m.angular();
    dJq_right << dJq_right_m.linear(), dJq_right_m.angular();

    // Compute the joint acceleration: qdd = Jinv * (a - Jd * qd)
    std::vector<Eigen::VectorXd> joint_accelerations;
    joint_accelerations.emplace_back(Jpinv_left * (left_hand_acc - dJq_left));
    joint_accelerations.emplace_back(Jpinv_right * (right_hand_acc - dJq_right));

    return Joints(joint_accelerations);
  }

  Joints
  InverseKinematicsTalos::GetAllJointVelocities(
      const EndeffectorsVel &vel_B, const Eigen::VectorXd &q) const
  {
    std::vector<Eigen::VectorXd> pos;
    pos.emplace_back(q.head<7>());
    pos.emplace_back(q.tail<7>());
    Joints joint_positions(pos);
    return GetAllJointVelocities(vel_B, joint_positions);
  }

  Joints
  InverseKinematicsTalos::GetAllJointAccelerations(
      const EndeffectorsAcc &acc_B, const Eigen::VectorXd &q,
      const Eigen::VectorXd &qd) const
  {
    std::vector<Eigen::VectorXd> pos;
    pos.emplace_back(q.head<7>());
    pos.emplace_back(q.tail<7>());
    Joints joint_positions(pos);

    std::vector<Eigen::VectorXd> vel;
    vel.emplace_back(qd.head<7>());
    vel.emplace_back(qd.tail<7>());
    Joints joint_velocities(vel);

    return GetAllJointAccelerations(acc_B, joint_positions, joint_velocities);
  }

  Eigen::MatrixXd
  InverseKinematicsTalos::GetFrameJacobian(int id) const
  {
    if (not talos_model_ or not talos_data_)
      return Eigen::MatrixXd::Zero(1, 1);

    // Initialize the Jacobian matrix with 0s (needed)
    Eigen::MatrixXd J(6, talos_model_->nv);
    J.setZero();
    // Compute the jacobian
    pinocchio::getFrameJacobian(*talos_model_, *talos_data_, id,
				pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
				J);
    return J;
  }

  Eigen::MatrixXd
  InverseKinematicsTalos::ComputePseudoInverse(const Eigen::MatrixXd &J,
					       const Eigen::MatrixXd &W) const
  {
    // Jpinv = W^{-1}(J^T(JW^{-1}J^T)^{-1})
    return W.inverse() * (J.transpose() * (J * W.inverse() * J.transpose()).inverse());
  }
  } /* namespace xpp */

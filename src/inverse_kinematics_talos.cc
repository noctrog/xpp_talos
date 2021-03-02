#include <cmath>
#include <iostream>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <xpp_talos/inverse_kinematics_talos.h>

#include <xpp_states/endeffector_mappings.h>

#include <ros/ros.h>
#include <ros/package.h>

namespace xpp {

  InverseKinematicsTalos::InverseKinematicsTalos()
  {
    // Create model and data objects
    talos_model_.reset(new pinocchio::Model);
    talos_data_.reset(new pinocchio::Data);

    // Load robot model
    ROS_INFO("Loading URDF model...");
    std::string xpp_talos_path = ros::package::getPath("xpp_talos");
    if (xpp_talos_path.size() == 0) {
      ROS_ERROR("You need to install the xpp_talos package: https://github.com/noctrog/xpp_talos");
      exit(-1);
    }
    std::string urdf_path = xpp_talos_path + "/urdf/talos_full_legs_v2.urdf";
    std::cout << urdf_path << std::endl;
    // JointModelFreeFlyer indicates that the root of the robot is not fixed to the world
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), *talos_model_);
    ROS_INFO("Pinocchio model loaded success, robot name: %s", talos_model_->name.c_str());

    // Initialize pinocchio model data
    *talos_data_ = pinocchio::Data(*talos_model_);

    left_sole_id = talos_model_->getJointId("left_sole_link");
    right_sole_id = talos_model_->getJointId("right_sole_link");
  }

  Joints 
  InverseKinematicsTalos::GetAllJointAngles(const EndeffectorsPos& pos_B) const
  {
    using namespace biped;
    std::vector<Eigen::VectorXd> q_vec;

    // make sure always exactly 2 elements
    auto x_biped_B = pos_B.ToImpl();
    x_biped_B.resize(2, x_biped_B.front());

    /* TODO: Los vectores a la base <02-11-20, Ramón Calvo González> */
    q_vec.emplace_back(legs.GetJointAngles(LEFTLEG, 
					   x_biped_B.at(L) + Vector3d(0.0, 0.0, 0.0)));
    q_vec.emplace_back(legs.GetJointAngles(RIGHTLEG, 
					   x_biped_B.at(R) + Vector3d(0.0, 0.0, 0.0)));

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

    /* TODO: Los vectores a la base <02-11-20, Ramón Calvo González> */
    q_vec.emplace_back(legs.GetJointAngles(LEFTLEG, 
					   x_biped_B.at(L) + Vector3d(0.0, 0.0, 0.0),
					   rot_B.at(L)));
    q_vec.emplace_back(legs.GetJointAngles(RIGHTLEG, 
					   x_biped_B.at(R) + Vector3d(0.0, 0.0, 0.0),
					   rot_B.at(R)));

    return Joints(q_vec);
  }

  Joints
  InverseKinematicsTalos::GetAllJointVelocities(const EndeffectorsVel &vel_B,
			       const Joints &pos_j) const
  {
    // Retrieve the end effectors velocities (towr does not calculate orientations)
    Eigen::VectorXd left_sole_vel, right_sole_vel;
    left_sole_vel << vel_B.at(0), Eigen::VectorXd::Zero(3);
    right_sole_vel << vel_B.at(1), Eigen::VectorXd::Zero(3);

    // Retrieve the current robot state (spatial position)
    // XYZRPY of the base link do not matter
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6 + pos_j.GetNumJoints());
    for (size_t i = 0; i < pos_j.GetNumJoints(); ++i)
      q(6+i) = pos_j.GetJoint(i);

    // Compute contact Jacobians
    Eigen::MatrixXd J_left_sole = ComputeFrameJacobian(q, left_sole_id);
    Eigen::MatrixXd J_right_sole = ComputeFrameJacobian(q, right_sole_id);

    // Slice to obtain the Jacobian for each kinematic cheain (each leg)
    Eigen::MatrixXd J_left_slice = J_left_sole.block(0, 6, 6, 6);
    Eigen::MatrixXd J_right_slice = J_left_sole.block(0, 12, 6, 6);

    // Compute velocities: qd = Jinv * v
    std::vector<Eigen::VectorXd> joint_velocities;
    joint_velocities.emplace_back(J_left_slice.inverse() * left_sole_vel);
    joint_velocities.emplace_back(J_right_slice.inverse() * right_sole_vel);

    return Joints(joint_velocities);
  }

  Joints
  InverseKinematicsTalos::GetAllJointAccelerations(const EndeffectorsAcc &acc_B,
						   const Joints &pos_j,
						   const Joints &vel_j) const
  {
    // Retrieve the end effector accelerations (towr does not calculate orientations)
    Eigen::VectorXd left_sole_acc, right_sole_acc;
    left_sole_acc << acc_B.at(0), Eigen::VectorXd::Zero(3);
    right_sole_acc << acc_B.at(0), Eigen::VectorXd::Zero(3);

    // Retrieve the current robot state (spatial position and velocity)
    // TODO: XYZRPY velocities matter!!!!!!
    Eigen::VectorXd q  = Eigen::VectorXd::Zero(6 + pos_j.GetNumJoints());
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(6 + vel_j.GetNumJoints());
    for (size_t i = 0; i < pos_j.GetNumJoints(); ++i) {
      q(6+i)  = pos_j.GetJoint(i);
      qd(6+i) = vel_j.GetJoint(i);
    }

    // Compute all the jacobians needed and slice
    Eigen::MatrixXd J_left = ComputeFrameJacobian(q, left_sole_id).block(0, 6, 6, 6);
    Eigen::MatrixXd J_right = ComputeFrameJacobian(q, right_sole_id).block(0, 12, 6, 6);
    Eigen::MatrixXd dJ_left = ComputeFrameJacobianTimeDerivative(q, qd, left_sole_id).block(0, 6, 6, 6);
    Eigen::MatrixXd dJ_right = ComputeFrameJacobianTimeDerivative(q, qd, right_sole_id).block(0, 12, 6, 6);

    // Compute the joint acceleration: qdd = Jinv * (a - Jd * qd)
    std::vector<Eigen::VectorXd> joint_accelerations;
    joint_accelerations.emplace_back(J_left.inverse() * (left_sole_acc - dJ_left * qd));
    joint_accelerations.emplace_back(J_right.inverse() * (right_sole_acc - dJ_right * qd));

    return Joints(joint_accelerations);
  }

  Eigen::MatrixXd
  InverseKinematicsTalos::ComputeFrameJacobian(Eigen::VectorXd q, int id) const
  {
    if (not talos_model_ or not talos_data_) return Eigen::MatrixXd::Zero(1,1);

    // Initialize the Jacobian matrix with 0s (needed)
    Eigen::MatrixXd J(6, talos_model_->nv);
    J.setZero();
    // Compute the jacobian
    pinocchio::computeFrameJacobian(*talos_model_, *talos_data_, q, id,
				    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
				    J);
    return J;
  }

  Eigen::MatrixXd
  InverseKinematicsTalos::ComputeFrameJacobianTimeDerivative(
      Eigen::VectorXd q, Eigen::VectorXd qd, int id) const
  {
    if (not talos_model_ or not talos_data_) return Eigen::MatrixXd::Zero(1,1);

    // Computes the time derivative of the jacobian and saves it in
    // talos_data_, needed to then calculate the time derivative of
    // the frame jacobian
    pinocchio::computeJointJacobiansTimeVariation(*talos_model_, *talos_data_, q, qd);

    // Computes the requested jacobian with respect to the frame id
    Eigen::MatrixXd dJ(6, talos_model_->nv);
    dJ.setZero();
    pinocchio::getFrameJacobianTimeVariation(*talos_model_, *talos_data_, id,
					     pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
					     dJ);
    // Return the result
    return dJ;
  }

} /* namespace xpp */

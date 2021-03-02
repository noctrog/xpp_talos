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
    // TODO pensar un poco: se necesitan las velocidades de la base (tanto posicion como orientacion)
    // Compute the Jacobian for pos_j
    int left_sole_id = talos_model_->getJointId("left_sole_link");
    int right_sole_id = talos_model_->getJointId("right_sole_link");

    // Retrieve the end effectors velocities (towr does not calculate orientations)
    Eigen::VectorXd left_sole_vel, right_sole_vel;
    left_sole_vel << vel_B.at(0), Eigen::VectorXd::Zero(3);
    right_sole_vel << vel_B.at(1), Eigen::VectorXd::Zero(3);

    // Retrieve the current robot state (spatial position)
    // TODO: XYZRPY does not matter??
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6 + pos_j.GetNumJoints());
    for (size_t i = 0; i < pos_j.GetNumJoints(); ++i)
      q(6+i) = pos_j.GetJoint(i);

    // Compute contact Jacobians
    Eigen::MatrixXd J_left_sole(6, talos_model_->nv);
    J_left_sole.setZero();
    pinocchio::computeFrameJacobian(*talos_model_, *talos_data_, q, left_sole_id,
				    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
				    J_left_sole);
    Eigen::MatrixXd J_right_sole(6, talos_model_->nv);
    J_right_sole.setZero();
    pinocchio::computeFrameJacobian(*talos_model_, *talos_data_, q, right_sole_id,
				    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
				    J_right_sole);

    // Slice to obtain the Jacobian for each kinematic cheain (each leg)
    Eigen::MatrixXd J_left_slice = J_left_sole.block(0, 6, 6, 6);
    Eigen::MatrixXd J_right_slice = J_left_sole.block(0, 12, 6, 6);

    // Compute velocities: qd = Jinv * v
    std::vector<Eigen::VectorXd> joint_velocities;
    joint_velocities.emplace_back(J_left_sole.inverse() * left_sole_vel);
    joint_velocities.emplace_back(J_right_sole.inverse() * right_sole_vel);

    return Joints(joint_velocities);
  }

  Joints InverseKinematicsTalos::GetAllJointAccelerations(const EndeffectorsAcc &acc_B,
				  const Joints &pos_j,
                                  const Joints &vel_j) const
  {
    
  }

  } /* namespace xpp */

#include <xpp_talos/inverse_kinematics_talos.h>

#include <cmath>
#include <iostream>

#include <xpp_states/endeffector_mappings.h>

namespace xpp {

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

		// Fixed joints
		//  1 - torso_1_joint torso_2_joint
		//  3 - arm_left_1_joint arm_left_2_joint arm_left_3_joint arm_left_4_joint
		//  7 - arm_left_5_joint arm_left_6_joint arm_left_7_joint gripper_left_joint
		// 11 - arm_right_1_joint arm_right_2_joint arm_right_3_joint arm_right_4_joint
		// 15 - arm_right_5_joint arm_right_6_joint arm_right_7_joint gripper_right_joint
		// 19 - head_1_joint head_2_joint
		auto q_fixed = Eigen::VectorXd::Zero(59 - 12);
		q_vec.push_back(q_fixed);
		return Joints(q_vec);
	}

} /* namespace xpp */

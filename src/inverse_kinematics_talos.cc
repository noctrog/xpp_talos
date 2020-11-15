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

    Joints a(q_vec);
    return Joints(q_vec);
  }

} /* namespace xpp */

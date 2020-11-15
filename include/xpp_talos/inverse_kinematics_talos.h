#ifndef INVERSE_KINEMATICS_TALOS_H_
#define INVERSE_KINEMATICS_TALOS_H_

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_talos/taloslegs_inverse_kinematics.h>

namespace xpp {

  /**
   * @brief Inverse Kinematics for the Talos legs.
   */
  class InverseKinematicsTalos : public InverseKinematics {
  public:
    InverseKinematicsTalos() = default;
    virtual ~InverseKinematicsTalos() = default;

    /**
     * @brief Returns joint angles to reach for a specific foot position.
     * @param pos_B  3D-position of the foot expressed in the base frame (B).
     */
    Joints GetAllJointAngles(const EndeffectorsPos& pos_B) const override;

    /**
     * @brief Number of endeffectors (2 feet).
     */
    int GetEECount() const override { return 2; };

  private:
    TalosLegsInverseKinematics legs;
  };

} /* namespace xpp  */

#endif /* end of include guard: INVERSE_KINEMATICS_TALOS_H_ */

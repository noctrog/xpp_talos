#ifndef INVERSE_KINEMATICS_TALOS_H_
#define INVERSE_KINEMATICS_TALOS_H_

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 30

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <xpp_vis/inverse_kinematics.h>
#include <xpp_talos/taloslegs_inverse_kinematics.h>

namespace xpp {

  using EndeffectorsRot = Endeffectors<Eigen::Matrix3d>;

  /**
   * @brief Inverse Kinematics for the Talos legs.
   */
  class InverseKinematicsTalos : public InverseKinematics {
  public:
    using Ptr = std::shared_ptr<InverseKinematicsTalos>;

    InverseKinematicsTalos();
    virtual ~InverseKinematicsTalos() = default;

    /**
     * @brief Returns joint angles to reach for a specific foot position.
     * @param pos_B  3D-position of the foot expressed in the base frame (B).
     */
    Joints GetAllJointAngles(const EndeffectorsPos& pos_B) const override;

    /**
     * @brief Return joint angles to reach for a specific foot position.
     * @param pos_B  3D-position of the foot expressed in the base frame (B)
     * @param rot_B  Orientation of the end effector with respect to the base frame (B)
     */
    Joints GetAllJointAngles(const EndeffectorsPos& pos_B,
			     const EndeffectorsRot& rot_B) const;

    /**
     * @brief Return joint velocities.
     * @param vel_B  3D-velocity of the foot expressed in the base frame (B)
     * @param pos_j  Current positions for every joint.
     */
    Joints GetAllJointVelocities(const EndeffectorsVel &vel_B,
				 const Joints &pos_j) const;

    /**
     * @brief Return joint velocities.
     * @param acc_B  3D-acceleration of the foot expressed in the base frame (B)
     * @param pos_j  Current positions for every joint.
     * @param vel_j  Current velocities for every joint.
     */
    Joints GetAllJointAccelerations(const EndeffectorsAcc &acc_B,
				    const Joints &pos_j,
				    const Joints &vel_j) const;
    /**
     * @brief Number of endeffectors (2 feet).
     */
    int GetEECount() const override { return 2; };

  private:
    TalosLegsInverseKinematics legs;

    typedef pinocchio::Model Model;
    typedef pinocchio::Data Data;
    typedef std::shared_ptr<Model> ModelPtr;
    typedef std::shared_ptr<Data> DataPtr;
    ModelPtr talos_model_;
    DataPtr talos_data_;
  };

} /* namespace xpp  */

#endif /* end of include guard: INVERSE_KINEMATICS_TALOS_H_ */

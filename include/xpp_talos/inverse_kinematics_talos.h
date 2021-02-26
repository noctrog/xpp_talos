#ifndef INVERSE_KINEMATICS_TALOS_H_
#define INVERSE_KINEMATICS_TALOS_H_

#include <xpp_vis/inverse_kinematics.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>

namespace xpp {

  using EndeffectorsRot = Endeffectors<Eigen::Matrix3d>;

  enum TALOS_ARMS_JOINTS{
    AL1=0, AL2, AL3, AL4, AL5, AL6, AL7,
    AR1, AR2, AR3, AR4, AR5, AR6, AR7,
  };

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
     * @brief Number of endeffectors (2 feet).
     */
    int GetEECount() const override { return 2; };

  private:
    typedef KDL::ChainFkSolverPos FkSolverPos;
    typedef std::unique_ptr<FkSolverPos> FkSolverPosPtr;
    typedef KDL::ChainIkSolverPos IkSolverPos;
    typedef std::unique_ptr<IkSolverPos> IkSolverPosPtr;
    typedef KDL::ChainIkSolverVel IkSolverVel;
    typedef std::unique_ptr<IkSolverVel> IkSolverVelPtr;
    typedef KDL::ChainIkSolverAcc IkSolverAcc;
    typedef std::unique_ptr<IkSolverAcc> IkSolverAccPtr;

    // Kinematic tree and chains
    KDL::Tree kinematic_tree_;
    KDL::Chain left_arm_chain_;
    KDL::Chain right_arm_chain_;

    // Forward kinematics solvers
    FkSolverPosPtr left_arm_fk_pos_, right_arm_fk_pos_;

    // Inverse kinematics solvers
    IkSolverPosPtr left_arm_ik_pos_, right_arm_ik_pos_;
    IkSolverVelPtr left_arm_ik_vel_, right_arm_ik_vel_;
    IkSolverAccPtr left_arm_ik_acc_, right_arm_ik_acc_;

    // Last solved positions, used for warm starting the ik solver
    typedef std::unique_ptr<KDL::JntArray> JntArrayPtr;
    JntArrayPtr left_arm_last_pos_, right_arm_last_pos_;
  };

} /* namespace xpp  */

#endif /* end of include guard: INVERSE_KINEMATICS_TALOS_H_ */

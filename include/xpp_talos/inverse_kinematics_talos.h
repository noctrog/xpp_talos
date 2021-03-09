#ifndef INVERSE_KINEMATICS_TALOS_H_
#define INVERSE_KINEMATICS_TALOS_H_
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 30

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

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
     * @brief Return joint velocities.
     * @param vel_B  3D-velocity of the foot expressed in the base frame (B)
     * @param q  Current positions for every joint.
     */
    Joints GetAllJointVelocities(const EndeffectorsVel &vel_B,
				 const Eigen::VectorXd &q) const;

    /**
     * @brief Return joint velocities.
     * @param acc_B  3D-acceleration of the foot expressed in the base frame (B)
     * @param q  Current positions for every joint.
     * @param qd  Current velocities for every joint.
     */
    Joints GetAllJointAccelerations(const EndeffectorsAcc &acc_B,
				    const Eigen::VectorXd &q,
				    const Eigen::VectorXd &qd) const;

    /**
     * @brief Number of endeffectors (2 feet).
     */
    int GetEECount() const override { return 2; };

  private:
    /**
     * @brief Returns the Jacobian of the frame specified.
     * 
     * Before calling this method, pinocchio::computeJointJacobians
     * and pinocchio::updateFramePlacements must be called first.
     * 
     */
    Eigen::MatrixXd
    GetFrameJacobian(int id) const;

    /** 
     * @brief Computes the Moore-Penrose pseudoinverse
     */
    Eigen::MatrixXd
    ComputePseudoInverse(const Eigen::MatrixXd& J,
			 const Eigen::MatrixXd& W) const;

    typedef KDL::ChainFkSolverPos FkSolverPos;
    typedef std::unique_ptr<FkSolverPos> FkSolverPosPtr;
    typedef KDL::ChainIkSolverPos IkSolverPos;
    typedef std::unique_ptr<IkSolverPos> IkSolverPosPtr;
    typedef KDL::ChainIkSolverVel IkSolverVel;
    typedef std::unique_ptr<IkSolverVel> IkSolverVelPtr;

    // Kinematic tree and chains
    KDL::Tree kinematic_tree_;
    KDL::Chain left_arm_chain_;
    KDL::Chain right_arm_chain_;

    // Forward kinematics solvers
    FkSolverPosPtr left_arm_fk_pos_, right_arm_fk_pos_;

    // Inverse kinematics solvers
    IkSolverPosPtr left_arm_ik_pos_, right_arm_ik_pos_;
    IkSolverVelPtr left_arm_ik_vel_, right_arm_ik_vel_;

    // Last solved positions, used for warm starting the ik solver
    typedef std::unique_ptr<KDL::JntArray> JntArrayPtr;
    JntArrayPtr left_arm_last_pos_, right_arm_last_pos_;

    // Pinocchio robot model
    typedef pinocchio::Model Model;
    typedef pinocchio::Data Data;
    typedef std::shared_ptr<Model> ModelPtr;
    typedef std::shared_ptr<Data> DataPtr;
    ModelPtr talos_model_;
    DataPtr talos_data_;

    int left_hand_id, right_hand_id;
  };

} /* namespace xpp  */

#endif /* end of include guard: INVERSE_KINEMATICS_TALOS_H_ */

#include <xpp_talos/taloslegs_inverse_kinematics.h>
#include <xpp_states/cartesian_declarations.h>

#define IKFAST_CLIBRARY
#define IKFAST_NO_MAIN
#define IKFAST_NAMESPACE

#include "ik_left_leg_6d.cc"
#include "ik_right_leg_6d.cc"

namespace xpp {

	Eigen::VectorXd 
	TalosLegsInverseKinematics::GetJointAngles(const TalosLeg leg,
											   const Vector3d& ee_pos_H,
											   const Matrix3d& ee_or) const
	{
		/* TODO: <02-11-20, Ramón Calvo González> */

		// Final solution q
		Eigen::VectorXd q(6); q.setZero();
		switch (leg) {
			case LEFTLEG:
				{
					using namespace IKFAST_NAMESPACE_LEFT;
					// Solution container, stores all possible solutions
					ikfast_left::IkSolutionList<IkReal> solutions;
					// Representation for IK solver
					IkReal eerot[9], eetrans[3];
					bool bSuccess = false;
					// Create en effector desired 4x4 matrix (last row is 0 0 0 1 always)
					eerot[0] = ee_or(0,0); eerot[1] = ee_or(0,1); eerot[2] = ee_or(0,2); eetrans[0] = ee_pos_H.x();
					eerot[3] = ee_or(1,0); eerot[4] = ee_or(1,1); eerot[5] = ee_or(1,2); eetrans[1] = ee_pos_H.y();
					eerot[6] = ee_or(2,0); eerot[7] = ee_or(2,1); eerot[8] = ee_or(2,2); eetrans[2] = ee_pos_H.z();
					bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);

					// Failed to get a solution, return 0 for all joints
					if( !bSuccess ) {
						fprintf(stderr,"Failed to get ik solution\n");
						return q;
					}

					// printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
					// std::vector<IkReal> solvalues(GetNumJoints());
					// for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
					// const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
					// printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
					// std::vector<IkReal> vsolfree(sol.GetFree().size());
					// sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
					// for( std::size_t j = 0; j < solvalues.size(); ++j)
					// printf("%.15f, ", solvalues[j]);
					// printf("\n");
					// }
				}
				break;
			case RIGHTLEG:
				/* TODO: <02-11-20, Ramón Calvo González> */
				break;
			default:
				break;
		}
	}
}

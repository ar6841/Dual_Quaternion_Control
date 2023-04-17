/*
Control using dual quaternion differrence error. 
*/

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Dual_Quaternion_Forward_Kinematics/dualquat/dualquat.h"

namespace Controller
{

template<typename T>
Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic>
pose_difference_controller(const Kinematics::RobotLinks<T>& Robot, const dualquat::DualQuaternion<T>& desired_pose, 
const T& DT, const T& total_time, const T& tuning_param)
{
    // Avoid excessive stack memory usage by not declaring any variables inside.

    Eigen::Matrix<T,8,8> K(tuning_param*Eigen::MatrixXd::Identity(8, 8));

    dualquat::DualQuaternion<T> current, error;

    int N = ceil(total_time/DT); 

    int n = Robot.getNumJointsTotal()

    Eigen::Matrix<T,8, Eigen::Dynamic> = Jacobian(8,n)

    Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic> = THETA_TABLE(n,N);

    Eigen::Matrix<T,Eigen::Dynamic, 1> = theta_dot(n,1), theta_curr(n,1), theta_next(n,1);

    for(int i=0; i<N; i++) //use for loop to avoid infinite till error converges
    {
        current = Robot.ComputeForwardKinematics();

        error = desired_pose - current;

        Jacobian = ComputeJacobian(Robot);

        theta_dot = Jacobian*K*error;

        theta_curr = Robot.getJointVector();

        theta_next = dt*theta_dot + theta_curr;

        Robot.setJointVector(theta_next);

        THETA_TABLE.col(i) = theta_next;
  
    }

    return THETA_TABLE;

}
} //namespace Controller
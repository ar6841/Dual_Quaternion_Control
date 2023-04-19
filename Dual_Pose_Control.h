/*
Control using dual quaternion differrence error. 
*/

namespace Controller
{

template<typename T>
Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic>
pose_difference_controller(Kinematics::RobotLinks<T>& Robot, const dualquat::DualQuaternion<T>& desired_pose, 
const T& DT, const T& total_time, const T& tuning_param)
{
    // Avoid excessive stack memory usage by not declaring any variables inside.

    Eigen::Matrix<T,8,8> K(tuning_param*Eigen::MatrixXd::Identity(8, 8));

    dualquat::DualQuaternion<T> current, error;

    int N = ceil(total_time/DT); 

    int n = Robot.getNumJointsTotal();

    Eigen::Matrix<T,8, Eigen::Dynamic> Jacobian(8,n);

    Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic> THETA_TABLE(n,N);

    Eigen::Matrix<T,Eigen::Dynamic, 1> theta_dot(n,1), theta_curr(n,1), theta_next(n,1);

    for(int i=0; i<N; i++) //use for loop to avoid infinite till error converges
    {
        current = Robot.ComputeForwardKinematics();

        error = desired_pose - current;

        Jacobian = Kinematics::ComputeJacobian(Robot);

        // pinv() method is slow

        theta_dot = -(Jacobian.transpose())*(K*error); // these brackets are SUPER important

        // Change the operators within dualquat_base to accomodate no brackets

        theta_curr = Robot.getJointVec();

        theta_next = (DT*theta_dot + theta_curr);

        Robot.setJointVec(theta_next);

        THETA_TABLE.col(i) = theta_next.array().round();
  
    }

    return THETA_TABLE;

}
} //namespace Controller
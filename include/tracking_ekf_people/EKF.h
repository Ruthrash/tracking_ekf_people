#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <ros/ros.h>

class EKF
{
public:
    EKF();
    ~EKF();
    EKF(const int &m_, const int &n_, ros::NodeHandle &node);


protected:

private:
    bool is_initialized = false; 
	Eigen::VectorXd current_state;/*@brief  [x y z x_dot y_dot l w h]*/
	Eigen::VectorXd y;
	Eigen::MatrixXd current_covariance;
	Eigen::VectorXd prior_belief;
    Eigen::MatrixXd prior_covariance;
	Eigen::MatrixXd A;/*@brief motion model*/
	Eigen::MatrixXd C;/*@brief observation model*/
	Eigen::MatrixXd Q;/*@brief process noise covariance*/
	Eigen::MatrixXd R;/*@brief measurement noise covariance*/

    int m; /*@brief observation vector dimension  */
    int n; /*@brief state vector dimension  */

    double dt; /*@brief time difference for motion model*/
    
    ros::Time begin;/*@brief ros::Time variable used to calculate dt for motion model*/
    void Predictor();
    void Corrector(); 


    
};


#endif
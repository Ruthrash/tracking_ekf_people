#include "tracking_ekf_people/EKF.h"


EKF::EKF() {}
EKF::~EKF() {}

EKF::EKF(const int &m_, const int &n_, ros::NodeHandle &node): m{m_},  n{n_}
{
    Eigen::VectorXd temp_vec(n);
    temp_vec << 0.0,0.0,0.0,0.0,0.0,2.0,2.0,2.0; //initial state kept randomly   
    prior_belief = temp_vec;
    std::cout<<"Initial State:\n"<< prior_belief <<"\n";  
    temp_vec << 10000000.0,10000000.0,10000000.0,10000000.0,10000000.0,10000000.0,1000000.0,1000000.0; //initial covariance of the prior. Keep this very large for the estimate to converge quickly
    prior_covariance = temp_vec.asDiagonal();
    std::cout<<"Initial posterior covaraince\n"<<prior_covariance<<"\n";

    A = Eigen::MatrixXd::Identity(n,n);
    A(0,3) = dt;
    A(2,4) = dt;
    std::cout <<"Motion Model Matrix A:\n"<<A<<"\n";

    C = Eigen::MatrixXd::Zero(m,n);
    C.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);
    C.block<3,3>(3,5) = Eigen::MatrixXd::Identity(3,3);
    std::cout<<"Measurement Model Matrix C:\n"<<C<<"\n";

    float q, r,v; 
    node.getParam("prediction_noise",q);
    node.getParam("observation_noise",r);
    node.getParam("velocity_noise",v);
     
    //Set Q and R matrices
    Q = q*Eigen::MatrixXd::Identity(n,n);//Prediction noise covariance
    Q(3,3) = v;
    Q(4,4) = v;
    Q(n-3,n-3) = 0.1;
    Q(n-2,n-2) = 0.1;
    Q(n-1,n-1) = 0.1;
    std::cout<<"Prediction noise covariance Matrix Q:\n"<<Q<<"\n";
    R = r*Eigen::MatrixXd::Identity(m,m);//observation noise covariance
    std::cout<<" Observation noise covariance matrix R:\n"<<R<<"\n";
    is_initialized = true;
}

void EKF::Predictor()
{
    if(is_initialized)
    {
        ros::Time now = ros::Time::now();
        dt = now.toSec() - begin.toSec();

        A(0,3) = dt; 
        A(2,4) = dt;

        prior_belief = A*current_state;
        prior_covariance = A*current_covariance*A.transpose() + Q;
    }
}

void EKF::Corrector()
{
    //Compute Kalman gain
    Eigen::MatrixXd kalman_gain, temp_mat;

    temp_mat = C*prior_covariance*C.transpose() + R;

    temp_mat = temp_mat.inverse();
    kalman_gain = prior_covariance*C.transpose()*temp_mat;

    //Correction step using the measurements
    temp_mat = Eigen::MatrixXd::Identity(n,n);
    temp_mat = temp_mat - kalman_gain*C; 
    current_covariance = temp_mat*prior_covariance;
    current_state = prior_belief + kalman_gain*(y - C*prior_belief);

    begin = ros::Time::now();//change begin time to start timing from when a new update is made 
}
// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.


#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include "estimator/InvariantSmoother.hpp"
#include "utility/CallFile.hpp"
#include "utility/SaveFile.hpp"


#include <sstream>
#include <iomanip>

//#define filechecking

InvariantSmoother estimator_IS;

int main()
{

    double dt=0.005;
    int starting_point = 1;
    std::string log_path_ = LOG_PATH;
    std::string data_path_ = std::string(DATA_PATH) + "/sensor_data.csv";
    bool SR = false;
    double slip_thr = 0.48;
//    double slip_thr = 0.4;
//    double slip_thr = 0.5;
    bool VCC = false;
    double cov_amplifier = 1;
    bool retraction_flag = false;


    int max_backpp_no = 1;
    double backpp_rate=0.5;
    int max_it_no = 1;
    double convergence_cond = 1e-3;
 
    double gyro_exp = -6, acc_exp = -2, slip_exp = -1.3, contact_exp = -4, encoder_exp = -8; // for paper
    double bg_exp = -10, ba_exp = -10;
//    double bg_exp = -7, ba_exp = -3;
    double pri_ori_exp = -8, pri_vel_exp = -8, pri_pos_exp = -8;
    double pri_bg_exp = -10, pri_ba_exp = -10;

	EstimatorCovariances estimator_covariances;

  estimator_covariances.cov_gyro_diagonal << pow(10, gyro_exp), pow(10, gyro_exp), pow(10, gyro_exp);
  estimator_covariances.cov_acc_diagonal << pow(10, acc_exp), pow(10, acc_exp), pow(10, acc_exp);
  estimator_covariances.cov_slip_diagonal << pow(10, slip_exp), pow(10, slip_exp), pow(10, slip_exp);
  estimator_covariances.cov_contact_diagonal << pow(10, contact_exp), pow(10, contact_exp), pow(10, contact_exp);
  estimator_covariances.cov_enc_diagonal << pow(10, encoder_exp), pow(10, encoder_exp), pow(10, encoder_exp);
  estimator_covariances.cov_bias_gyro_diagonal << pow(10, bg_exp), pow(10, bg_exp), pow(10, bg_exp);
  estimator_covariances.cov_bias_acc_diagonal << pow(10, ba_exp), pow(10, ba_exp), pow(10, ba_exp);
  estimator_covariances.cov_prior_orientation_diagonal << pow(10, pri_ori_exp), pow(10, pri_ori_exp), pow(10, pri_ori_exp);
  estimator_covariances.cov_prior_velocity_diagonal << pow(10, pri_vel_exp), pow(10, pri_vel_exp), pow(10, pri_vel_exp);
  estimator_covariances.cov_prior_position_diagonal << pow(10, pri_pos_exp), pow(10, pri_pos_exp), pow(10, pri_pos_exp);
  estimator_covariances.cov_prior_bias_gyro_diagonal << pow(10, pri_bg_exp), pow(10, pri_bg_exp), pow(10, pri_bg_exp);
  estimator_covariances.cov_prior_bias_acc_diagonal << pow(10, pri_ba_exp), pow(10, pri_ba_exp), pow(10, pri_ba_exp);



    Eigen::Matrix<double, 16, 1> initial_condition;
    initial_condition <<0.0,0.0, 0.0 ,// Px,Py,Pz
            1, 0, 0, 0, // quaternion w,x,y,z
            0, 0, 0,     //Vx, Vy, Vz
            0, 0, 0,     //bgx, bgy, bgz
            0, 0, 0;    //bax, bay, baz


    int sample_no=15;
    bool IS_flag = true;

////
    Eigen::Matrix<double,30,1> Sensor_;
    Sensor_.setZero();
    Eigen::Matrix<bool,4,1> Contact_;
    Contact_.setZero();
    ROBOT_STATES state_;
    MEAS_FORWARD_KINEMATICS forkin_set_;

        if(IS_flag){

            estimator_IS.estimator_common_struct_.leg_no = 4;

            std::cout << "--- Reading CSV Data ---" << std::endl;
            std::vector<CSVData> csv_data = CallFile::readCSV(data_path_);
            if (!csv_data.empty()) {
                std::cout << "Successfully read " << csv_data.size() << " rows." << std::endl;
                std::cout << "First row IMU Acc: " << csv_data[0].imu_acc.transpose() << std::endl;
            } else {
                std::cout << "Failed to read CSV or file is empty." << std::endl;
            }
            std::cout << "------------------------" << std::endl;

            estimator_IS.Optimization_Epsilon = convergence_cond;
            estimator_IS.Max_Iteration = max_it_no;
            estimator_IS.Max_backpropagate_num = max_backpp_no;
            estimator_IS.backppgn_rate = backpp_rate;

            estimator_IS.NUM_OF_TRASH_DATA = starting_point;
            estimator_IS.slip_rejection_mode = SR;
            estimator_IS.slip_threshold = slip_thr;
            estimator_IS.variable_contact_cov_mode=VCC;
            estimator_IS.cov_amplifier=cov_amplifier;

            estimator_IS.Retract_All_flag = false;
            //estimator_common_struct_.define(REAL_ROBOT);
            estimator_IS.Initialize(dt, estimator_covariances, initial_condition);

            std::vector<ROBOT_STATES> state_history;

            for (int time = 0; time < csv_data.size() ; time++){
                //cout<<"now step "<<time<<endl;
                Sensor_.segment(0,3) = csv_data.at(time).imu_ang_vel;
                Sensor_.segment(3,3) = csv_data.at(time).imu_acc;
                Sensor_.segment(6,12) = csv_data.at(time).jnt_pos;
                Sensor_.segment(18,12) = csv_data.at(time).jnt_vel;
                forkin_set_.forkin_position = csv_data.at(time).forkin_position;
                forkin_set_.forkin_jacobian = csv_data.at(time).forkin_jacobian;
                Contact_ = csv_data.at(time).contact_state.cast<bool>();

                std::cout << " [IS] process... " << time << " / " << csv_data.size() << std::endl;

                std::cout << " [IS] gyro... \n" << Sensor_.segment(0,3).transpose() << std::endl;
                std::cout << " [IS] acc... \n" << Sensor_.segment(3,3).transpose() << std::endl;
                std::cout << " [IS] jntpos... \n" << Sensor_.segment(6,12).transpose() << std::endl;
                std::cout << " [IS] jntvel... \n" << Sensor_.segment(18,12).transpose() << std::endl;
                std::cout << " [IS] contact... \n" << Contact_.transpose() << std::endl;
                std::cout << " [IS] forkin_position 1... \n" << forkin_set_.forkin_position.at(0).transpose() << std::endl;
                std::cout << " [IS] forkin_position 4... \n" << forkin_set_.forkin_position.at(3).transpose() << std::endl;
                std::cout << " [IS] forkin_jacobian 1... \n" << forkin_set_.forkin_jacobian.at(0).transpose() << std::endl;
                std::cout << " [IS] forkin_jacobian 4... \n" << forkin_set_.forkin_jacobian.at(3).transpose() << std::endl;

                estimator_IS.Onestep(Sensor_,Contact_,forkin_set_,state_);
                std::cout << " [IS] state_R... \n" << state_.Rotation << std::endl;
                std::cout << " [IS] state_v... \n" << state_.Velocity.transpose()<< std::endl;
                std::cout << " [IS] state_p... \n" << state_.Position.transpose() << std::endl;

                state_history.push_back(state_);
            }

            SaveFile::writeCSV(log_path_ + "/result.csv", state_history);
            // estimator_IS.DoSaveAll(log_path_);

        }


    return 0;


}

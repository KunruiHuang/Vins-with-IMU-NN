#pragma once
#include <ceres/ceres.h>
#include <ros/assert.h>

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "../parameters.h"
#include "../utility/utility.h"
#include "imu_neural_netwrok_base.h"
#include "integration_base.h"

class IMUNeuralNetworkFactor : public ceres::SizedCostFunction<3, 7, 9, 7, 9> {
 public:
  IMUNeuralNetworkFactor() = delete;
  IMUNeuralNetworkFactor(ImuNeuralNetworkBase *imu_neural_network_measurement)
      : imu_neural_network_measurement_(imu_neural_network_measurement) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                          parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4],
                          parameters[2][5]);

    Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
    Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);
    Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);

    residual = imu_neural_network_measurement_->evaluate(Pi, Qi, Vi, Bai, Bgi,
                                                         Pj, Qj, Vj, Baj, Bgj);

    Eigen::Matrix<double, 3, 3> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, 3, 3>>(
            imu_neural_network_measurement_->covariance_.inverse())
            .matrixL()
            .transpose();
    residual = sqrt_info * residual;
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    //修改jacobina函数
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>>
            jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();
        jacobian_pose_i.block<3, 3>(O_P, O_P) = -Eigen::Matrix3d::Identity();
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>>
            jacobian_speedbias_i(jacobians[1]);
        jacobian_speedbias_i.setZero();
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>>
            jacobian_pose_j(jacobians[2]);
        jacobian_pose_j.setZero();
        jacobian_pose_j.block<3, 3>(O_P, O_P) = Eigen::Matrix3d::Identity();
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>>
            jacobian_speedbias_j(jacobians[3]);
        jacobian_speedbias_j.setZero();
      }
    }
    return true;
  }
  ImuNeuralNetworkBase *imu_neural_network_measurement_;
  Eigen::Vector3d translation_;
  Eigen::Vector3d covariance_;
};

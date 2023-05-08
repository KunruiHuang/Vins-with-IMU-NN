#pragma once

#include <ceres/ceres.h>

#include "../parameters.h"
#include "../utility/utility.h"
using namespace Eigen;

class ImuNeuralNetworkBase {
 public:
  ImuNeuralNetworkBase(){};
  void setTranslationAndCov(const Eigen::Vector3d &translation,
                            const Eigen::Vector3d &covariance) {
    translation_(0) = translation(0);
    translation_(1) = translation(1);
    translation_(2) = translation(2);
    covariance_.setZero();
    covariance_(0, 0) = covariance[0];
    covariance_(1, 1) = covariance[1];
    covariance_(2, 2) = covariance[2];
    return;
  }

  Eigen::Matrix<double, 3, 1> evaluate(
      const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi,
      const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai,
      const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
      const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
      const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj) {
    Eigen::Matrix<double, 3, 1> residuals;
    residuals.block<3, 1>(0, 0) = Pj - Pi - translation_;
    ///////////////////////////////////////////////
  //  ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());

    // Eigen::Matrix<double, 3, 1> cam_pose_error = Pj - Pi;
    // ROS_INFO("camera_pose norm:%f", cam_pose_error.norm());
    // ROS_INFO("neural network output norm:%f", translation_.norm());
    ////////////////////////////////////////
    return residuals;
  }

  //   double dt;
  Eigen::Vector3d translation_;
  Eigen::Matrix<double, 3, 3> covariance_;
};

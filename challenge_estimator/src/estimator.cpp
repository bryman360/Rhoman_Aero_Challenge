#include <estimator.hpp>

void Estimator::ingMagMessage(double net_time, Eigen::Vector3d vars) {

}

void Estimator::ingAngMessage(double net_time, Eigen::Vector3d vars) {

}

void Estimator::spin(double net_time) {

}

double Estimator::interp1MDWL(Eigen::MatrixXd x, Eigen::MatrixXd y, double xq, bool dounwrap, uint32_t wraprange, std::string varargin) {

}

std::array<Eigen::Matrix3d, 2> Estimator::getR(double roll, double pitch, double yaw, std::string frame, std::string units) {
    
}
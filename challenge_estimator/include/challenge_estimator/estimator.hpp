#pragma once

#include <Eigen/Dense>

class Estimator {
public:
    Estimator() {};
    void ingMagMessage(double net_time, Eigen::Vector3d vals);
    void ingAngMessage(double net_time, Eigen::Vector3d vals);
    void spin(double net_time);
    bool have_new_mag_meas = false;
    Eigen::Vector2d getAvgAzEL() {return average_az_el;};
    Eigen::Vector2d getInstAzEL() {return instant_az_el;};
private:
    Eigen::MatrixXd mag_meas = Eigen::MatrixXd::Zero(100, 4); // Time, x, y, z
    uint32_t mag_i = 0;
    uint32_t max_mag_i = 100;
    bool mag_buff_full = false;

    Eigen::MatrixXd ang_meas = Eigen::MatrixXd::Zero(100, 4); // Time, r, p, y
    uint32_t ang_i = 0;
    uint32_t max_ang_i = 100;
    bool ang_buff_full = false;

    double sync_freq = 10;
    double sync_delay = 0.1;
    double sync_next_t = 0;
    Eigen::MatrixXd mag_loc_meas = Eigen::MatrixXd::Zero(100000, 7); // Time, r, p, y, x, y, z
    uint32_t mag_loc_i = 0;

    double next_est_time = 0;
    double next_est_freq = 1;
    uint32_t est_vals_to_use = 100;
    Eigen::Vector3d average_mag_vector = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d instant_mag_vector = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector2d average_az_el = Eigen::Vector2d(0, 0);
    Eigen::Vector2d instant_az_el = Eigen::Vector2d(0, 0);
    double nmag = 0;

    double interp1MDWL(Eigen::VectorXd x, Eigen::VectorXd y, double xq, bool dounwrap, uint32_t wraprange);
    Eigen::Matrix3d getR(double roll, double pitch, double yaw);
};


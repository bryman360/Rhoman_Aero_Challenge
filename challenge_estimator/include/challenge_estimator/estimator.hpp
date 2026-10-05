#pragma once

#include <Eigen/Dense>
#include <memory>

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
    uint32_t mag_i = 0;
    const uint32_t max_mag_i = 100;
    Eigen::MatrixXd mag_meas = Eigen::MatrixXd::Zero(max_mag_i, 4); // Time, x, y, z
    bool mag_buff_full = false;

    uint32_t ang_i = 0;
    const uint32_t max_ang_i = 100;
    Eigen::MatrixXd ang_meas = Eigen::MatrixXd::Zero(max_ang_i, 4); // Time, r, p, y
    bool ang_buff_full = false;

    const double sync_freq = 10;
    const double sync_delay_s = 0.1;
    double sync_next_t_s = 920;
    Eigen::MatrixXd mag_loc_meas = Eigen::MatrixXd::Zero(100000, 7); // Time, r, p, y, x, y, z
    uint32_t mag_loc_i = 0;

    double next_est_time_s = 940;
    const double next_est_freq = 1;
    const uint32_t est_vals_to_use = 100;
    Eigen::Vector3d average_mag_vector = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d instant_mag_vector = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector2d average_az_el = Eigen::Vector2d(0, 0); // Az, El
    Eigen::Vector2d instant_az_el = Eigen::Vector2d(0, 0); // Az, El
    double nmag = 0;
};

class EstimatorPtr {
public:
    EstimatorPtr() {};
    EstimatorPtr(std::shared_ptr<Estimator> ptr) : sharedPtr(ptr){};
    ~EstimatorPtr() {sharedPtr.reset();};
    std::shared_ptr<Estimator> operator->() {return sharedPtr;}
    void operator=(std::shared_ptr<Estimator> new_ptr) {sharedPtr=new_ptr;}
private:
    std::shared_ptr<Estimator> sharedPtr;
};

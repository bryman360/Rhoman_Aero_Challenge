#include <estimator.hpp>
#include <iostream>

double interp1MDWL(Eigen::VectorXd x, Eigen::VectorXd y, double xq, bool dounwrap, uint32_t wraprange);
double interp1(Eigen::VectorXd x, Eigen::VectorXd y, double xq);
Eigen::Matrix3d getR(double roll, double pitch, double yaw);
Eigen::MatrixXd sortMatrixByCol(Eigen::MatrixXd mat, uint32_t col);
double wrapTo360(double deg_val);

void Estimator::ingMagMessage(double net_time, Eigen::Vector3d vars) {
    if (mag_i >= max_mag_i) {
        mag_buff_full = true;
        mag_i = 0;
    }
    Eigen::RowVector4d new_row_data = Eigen::RowVector4d(net_time, vars[0], vars[1], vars[2]);
    mag_meas.row(mag_i) = new_row_data;
    mag_i++;
}

void Estimator::ingAngMessage(double net_time, Eigen::Vector3d vars) {
    if (ang_i >= max_ang_i) {
        ang_buff_full = true;
        ang_i = 0;
    }
    Eigen::RowVector4d new_row_data = Eigen::RowVector4d(net_time, vars[0], vars[1], vars[2]);
    ang_meas.row(ang_i) = new_row_data;
    ang_i++;
}

void Estimator::spin(double net_time_s) {
    if (net_time_s > sync_next_t_s) {
        sync_next_t_s += (1 / sync_freq);
        if (ang_buff_full && mag_buff_full) {
            Eigen::MatrixXd mag_sorted = sortMatrixByCol(mag_meas, 0);
            Eigen::MatrixXd ang_sorted = sortMatrixByCol(ang_meas, 0);
            double interp_time_us = (net_time_s - sync_delay) * 1000000;
            Eigen::Vector3d mag_interp = Eigen::Vector3d(
                interp1MDWL(mag_sorted.col(0), mag_sorted.col(1), interp_time_us, false, 360),
                interp1MDWL(mag_sorted.col(0), mag_sorted.col(2), interp_time_us, false, 360),
                interp1MDWL(mag_sorted.col(0), mag_sorted.col(3), interp_time_us, false, 360)
            );
            Eigen::Vector3d ang_interp = Eigen::Vector3d(
                interp1MDWL(ang_sorted.col(0), ang_sorted.col(1), interp_time_us, true, 180),
                interp1MDWL(ang_sorted.col(0), ang_sorted.col(2), interp_time_us, true, 180),
                interp1MDWL(ang_sorted.col(0), ang_sorted.col(3), interp_time_us, true, 180)
            );
            Eigen::RowVectorXd mag_loc_meas_row_data = Eigen::RowVectorXd::Zero(7);
            mag_loc_meas_row_data <<
                interp_time_us,
                ang_interp[0], ang_interp[1], ang_interp[2],
                mag_interp[0], mag_interp[1], mag_interp[2];
            mag_loc_meas.row(mag_loc_i) = mag_loc_meas_row_data;
            mag_loc_i++;
        }
    }

    have_new_mag_meas = false;
    if (net_time_s > next_est_time_s) {
        next_est_time_s += (1 / next_est_freq);
        if (mag_loc_i >= 10) {
            uint32_t nvals = std::min(mag_loc_i, est_vals_to_use);
            Eigen::MatrixXd Amat = Eigen::MatrixXd::Zero(3*nvals, 6);
            Eigen::MatrixXd bmat = Eigen::MatrixXd::Zero(3*nvals, 1);
            uint32_t istart = 0;
            for (int i = 0; i < int(nvals); i++) {
                uint32_t iml = ceil(random() % mag_loc_i);
                Eigen::MatrixXd att_ang_val = mag_loc_meas.block(iml, 1, 1, 3);
                Eigen::MatrixXd mag_val = mag_loc_meas.block(iml, 4, 1, 3);
                Eigen::Matrix3d Rb2l = getR(att_ang_val(0, 0), att_ang_val(0, 1), att_ang_val(0, 2));
                Eigen::Matrix3d Rm2b = getR(0, 0, -90);
                Eigen::MatrixXd net_mag = Rb2l * Rm2b * mag_val.transpose();

                Amat.row(istart)   << 1, 0, 0, Rb2l(0, 0), Rb2l(0, 1), Rb2l(0, 2);
                Amat.row(istart+1) << 0, 1, 0, Rb2l(1, 0), Rb2l(1, 1), Rb2l(1, 2);
                Amat.row(istart+2) << 0, 0, 1, Rb2l(2, 0), Rb2l(2, 1), Rb2l(2, 2);
                bmat(istart, 0)   = net_mag(0);
                bmat(istart+1, 0) = net_mag(1);
                bmat(istart+2, 0) = net_mag(2);
                istart += 3;
            }

            Eigen::MatrixXd mat_sol = (Amat.transpose() * Amat).inverse() * (Amat.transpose() * bmat);
            Eigen::Vector3d mag_sol = Eigen::Map<const Eigen::VectorXd>(mat_sol.transpose().data(), 3);

            instant_mag_vector = -mag_sol / mag_sol.norm();
            average_mag_vector = (nmag * average_mag_vector + instant_mag_vector) / (nmag + 1);
            average_mag_vector = average_mag_vector / average_mag_vector.norm();
            nmag++;

            have_new_mag_meas = true;
            average_az_el << net_time_s * 1000000, wrapTo360(std::atan2(average_mag_vector[1], average_mag_vector[0]) * 180/M_PI), std::atan(average_mag_vector[2]/average_mag_vector.segment(0, 2).norm()) * 180/M_PI;
            instant_az_el << net_time_s * 1000000, wrapTo360(std::atan2(instant_mag_vector[1], instant_mag_vector[0]) * 180/M_PI), std::atan(instant_mag_vector[2]/instant_mag_vector.segment(0, 2).norm()) * 180/M_PI;
        }
    }
}

double interp1MDWL(Eigen::VectorXd x, Eigen::VectorXd y, double xq, bool dounwrap, uint32_t wraprange) {
    if (xq < x.minCoeff() || x.size() == 1) {
        return y[0];
    } else if (xq > x.maxCoeff()) {
        return y[y.size() - 1];
    }

    Eigen::VectorXd y1 = y;
    if (dounwrap) {
        uint32_t cumulative_shift = 0;
        for (int i = 1; i < y.size(); i++) {
            double jump = y[i] - y[i-1];
            if (abs(jump) > wraprange) {
                uint32_t shift_count = floor(abs(jump/wraprange)) * (jump < 0 ? -1 : 1);
                cumulative_shift -= (shift_count * wraprange);
            }
            y1[i] = y[i] + cumulative_shift;
        }
    }

    return interp1(x, y1, xq);
}

double interp1(Eigen::VectorXd x, Eigen::VectorXd y, double xq) {
    for (int i = 0; i < x.size() - 1; i++) {
        if (xq >= x[i]) {
            double ref_ratio = (xq - x[i]) / (x[i+1] - x[i]);
            double y_diff = y[i+1] - y[i];
            return (y_diff * ref_ratio) + y[i];
        }
    }
    return y[y.size() - 1];
}

Eigen::Matrix3d getR(double roll, double pitch, double yaw) {
    Eigen::Matrix3d Rx, Ry, Rz;

    roll = roll * M_PI/180;
    pitch = pitch * M_PI/180;
    yaw = yaw * M_PI/180;

    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);
    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          sin(-pitch), 0, cos(pitch);
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;
    
    Eigen::Matrix3d R = Rx * Ry * Rz;
    return R;
}

Eigen::MatrixXd sortMatrixByCol(Eigen::MatrixXd mat, uint32_t col) {
    Eigen::MatrixXd sorted_mat = Eigen::MatrixXd::Zero(mat.rows(), mat.cols());
    std::pair<int, double> indexes_and_timestamps[mat.rows()];
    
    for (int i=0; i < mat.rows(); i++) {
        indexes_and_timestamps[i].first = i;
        indexes_and_timestamps[i].second = mat(i, col);
    }

    int n = sizeof(indexes_and_timestamps) / sizeof(indexes_and_timestamps[0]);

    std::sort(indexes_and_timestamps, indexes_and_timestamps + n, [](std::pair<int, double> a, std::pair<int, double> b) {
        return a.second < b.second;
    });

    for (int i=0; i < mat.rows(); i++) {
        sorted_mat.row(i) = mat.row(indexes_and_timestamps[i].first);
    }

    return sorted_mat;
}

double wrapTo360(double deg_val) {
    while (deg_val >= 360) {
        deg_val -= 360;
        if (abs(deg_val) < 0.0000001) {
            return 0.0;
        } 
    }
    while (deg_val < 0) {
        deg_val += 360;
        if (abs(deg_val) < 0.0000001) {
            return 0.0;
        } 
    }
    return deg_val;
}
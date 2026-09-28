#include <estimator.hpp>

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

void Estimator::spin(double net_time) {
    /*
    if net_time > sync_next_t
        sync_next_t = sync_next_t + (1/sync_freq)
        if ang_buff_full and mag_buff_full
            mag_sorted = sortrows(mag_meas, 1, "ascend")
            ang_sorted = sortrows(ang_meas, 1, "ascend")
            interp_time = net_time - sync_delay
            mag_loc_i++
            mag_interp = [interp1MDWL(mag_sorted[:, 1], mag_sorted[:, 2], interp_time, false, 360),
                          interp1MDWL(mag_sorted[:, 1], mag_sorted[:, 3], interp_time, false, 360),
                          interp1MDWL(mag_sorted[:, 1], mag_sorted[:, 4], interp_time, false, 360)]
            ang_interp = [interp1MDWL(ang_sorted[:, 1], ang_sorted[:, 2], interp_time, true, 180),
                          interp1MDWL(ang_sorted[:, 1], ang_sorted[:, 3], interp_time, true, 180),
                          interp1MDWL(ang_sorted[:, 1], ang_sorted[:, 4], interp_time, true, 180)]
            mag_loc_meas(mag_loc_i) = [interp_time, interp_ang, mag_interp]
    
    have_mag_meas = false
    if net_time > next_est_time
        next_est_time = next_est_time + (1/next_est_freq)
        if mag_loc_i > 10
            nvals = min(mag_loc_i, est_vals_to_use)
            Amat = zeros(3*nvals, 6)
            bmat = zeros(3*nvals, 1)
            istart = 2
            for n=1:nvals
                iml = ceil(rand*double(mag_loc_i))
                attval = mag_loc_meas[iml, 2:4]
                magval = mag_loc_meas[iml, 5:7]
                [Rb2l, ~] = getR(attval[1], attval[2], attval[3], 'NED', 'deg')
                [Rm2b, ~] = getR(0, 0, -90, 'NED', 'deg')
                netmeg = Rb2l * Rm2b * magval
                istart += 3
                Amat[istart:istart+2, :] = [1, 0, 0, Rb2l[1, :]; 0, 1, 0, Rb2l[2, :]; 0, 0, 1, Rb2l[3, :]]
                bmat[istart:istart+2, 1] = netmeg
            matsol = inv(Amat.T * Amat) * (Amat.T * bmat)
            magsol = matsol[1:3]
            
            instant_mag_vector = -magsol / norm(magsol)
            average_mag_vector = (nmag*average_mag_vector + instant_mag_vector) / (nmag + 1)
            average_mag_vector = average_mag_vector / norm(average_mag_vector)
            nmag++
            have_new_mag_measure = true
            average_az_el = [wrapTo360(atan2d(average_mag_vector[2], average_mag_vector[1])); atand(average_mag_vector[3]/norm(average_mag_vector[1:2]))]
            instant_az_el = [wrapTo360(atan2d(instant_mag_vector[2], instant_mag_vector[1])); atand(instant_mag_vector[3]/norm(instant_mag_vector[1:2]))]
    */
}

double Estimator::interp1MDWL(Eigen::VectorXd x, Eigen::VectorXd y, double xq, bool dounwrap, uint32_t wraprange) {
    if (xq < x.minCoeff()) {
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

    // return interp1(x, y1, xq, 'linear');
    return 0.0;
}

Eigen::Matrix3d Estimator::getR(double roll, double pitch, double yaw) {
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
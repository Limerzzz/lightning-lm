//
// Created by codex on 2026/2/25.
//

#ifndef LIGHTNING_INS_H
#define LIGHTNING_INS_H

#include <cstdint>

#include "common/eigen_types.h"

namespace lightning {

struct InsConfig {
    bool enable_fusion = true;
    bool use_llh = true;
    double base_longtitude = 0.0;
    double base_latitude = 0.0;
    double base_altitude = 0.0;

    double max_time_diff = 0.05;
    double rtk_other_noise_scale = 2.0;

    double rtk_fix_pos_noise = 1.0;
    double rtk_fix_ang_noise = 1.0;
    double rtk_other_pos_noise = 5.0;
    double rtk_other_ang_noise = 5.0;
};

struct InsMeasurement {
    double timestamp = 0.0;
    SE3 pose = SE3();
    Mat6d cov = Mat6d::Identity();
    uint8_t status = 0;
    bool valid = false;
};

}  // namespace lightning

#endif  // LIGHTNING_INS_H

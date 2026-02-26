//
// Created by codex on 2026/2/25.
//

#include "wrapper/ins_converter.h"

#include <algorithm>

#include "common/constant.h"
#include "utils/geodesy.h"
#include "wrapper/ros_utils.h"

namespace lightning {

namespace {
Mat6d BuildCov(double pos_noise, double ang_noise) {
    const double pos = std::max(pos_noise, 1e-6);
    const double ang = std::max(ang_noise, 1e-6);
    Mat6d cov = Mat6d::Zero();
    cov(0, 0) = pos * pos;
    cov(1, 1) = pos * pos;
    cov(2, 2) = pos * pos;
    cov(3, 3) = ang * ang;
    cov(4, 4) = ang * ang;
    cov(5, 5) = ang * ang;
    return cov;
}

}  // namespace

InsMeasurement ConvertLocalizationInfo(const bot_msg::msg::LocalizationInfo& msg, const InsConfig& cfg) {
    InsMeasurement out;
    out.timestamp = ToSec(msg.header.stamp);
    out.status = msg.rtk_status;
    out.valid = true;

    Vec3d enu;
    if (cfg.use_llh) {
        enu = geo::GeodeticToENU(msg.latitude, msg.longtitude, msg.altitude,
                                 cfg.base_latitude, cfg.base_longtitude, cfg.base_altitude);
    } else {
        enu = Vec3d(msg.east, msg.north, msg.up);
    }

    const double yaw_heading_deg = static_cast<double>(msg.yaw);
    const double pitch_deg = static_cast<double>(msg.pitch);
    const double roll_deg = static_cast<double>(msg.roll);

    const double yaw_enu = (90.0 - yaw_heading_deg) * constant::kDEG2RAD;
    const double pitch = pitch_deg * constant::kDEG2RAD;
    const double roll = roll_deg * constant::kDEG2RAD;

    Quatd q = AngAxisd(yaw_enu, Vec3d::UnitZ()) * AngAxisd(pitch, Vec3d::UnitY()) * AngAxisd(roll, Vec3d::UnitX());
    q.normalize();

    out.pose = SE3(q, enu);

    const bool is_fix = (out.status == 4);
    double pos_noise = is_fix ? cfg.rtk_fix_pos_noise : cfg.rtk_other_pos_noise * cfg.rtk_other_noise_scale;
    double ang_noise = is_fix ? cfg.rtk_fix_ang_noise : cfg.rtk_other_ang_noise * cfg.rtk_other_noise_scale;
    out.cov = BuildCov(pos_noise, ang_noise);

    return out;
}

}  // namespace lightning

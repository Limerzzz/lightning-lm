//
// Created by codex on 2026/2/25.
//

#ifndef LIGHTNING_GEODESY_H
#define LIGHTNING_GEODESY_H

#include "common/eigen_types.h"

namespace lightning::geo {

Vec3d GeodeticToECEF(double lat_rad, double lon_rad, double alt);
Vec3d GeodeticToENU(double lat_deg, double lon_deg, double alt,
                    double base_lat_deg, double base_lon_deg, double base_alt);

}  // namespace lightning::geo

#endif  // LIGHTNING_GEODESY_H

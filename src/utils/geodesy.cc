//
// Created by codex on 2026/2/25.
//

#include "utils/geodesy.h"

#include <cmath>

namespace lightning::geo {

namespace {
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);
constexpr double kDeg2Rad = M_PI / 180.0;
}  // namespace

Vec3d GeodeticToECEF(double lat_rad, double lon_rad, double alt) {
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);

    const double N = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);

    const double x = (N + alt) * cos_lat * cos_lon;
    const double y = (N + alt) * cos_lat * sin_lon;
    const double z = (N * (1.0 - kWgs84E2) + alt) * sin_lat;

    return Vec3d(x, y, z);
}

Vec3d GeodeticToENU(double lat_deg, double lon_deg, double alt,
                    double base_lat_deg, double base_lon_deg, double base_alt) {
    const double lat = lat_deg * kDeg2Rad;
    const double lon = lon_deg * kDeg2Rad;
    const double base_lat = base_lat_deg * kDeg2Rad;
    const double base_lon = base_lon_deg * kDeg2Rad;

    const Vec3d ecef = GeodeticToECEF(lat, lon, alt);
    const Vec3d ecef0 = GeodeticToECEF(base_lat, base_lon, base_alt);
    const Vec3d d = ecef - ecef0;

    const double sin_lat0 = std::sin(base_lat);
    const double cos_lat0 = std::cos(base_lat);
    const double sin_lon0 = std::sin(base_lon);
    const double cos_lon0 = std::cos(base_lon);

    const double east = -sin_lon0 * d.x() + cos_lon0 * d.y();
    const double north = -sin_lat0 * cos_lon0 * d.x() - sin_lat0 * sin_lon0 * d.y() + cos_lat0 * d.z();
    const double up = cos_lat0 * cos_lon0 * d.x() + cos_lat0 * sin_lon0 * d.y() + sin_lat0 * d.z();

    return Vec3d(east, north, up);
}

}  // namespace lightning::geo

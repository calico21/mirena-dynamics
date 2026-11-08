#pragma once

#include <cmath>

namespace mirena::GNSSConversion
{
    // WGS84 ellipsoid.
    inline constexpr double A = 6378137.0;         // semi-major axis, metres
    inline constexpr double E2 = 0.00669437999014; // first eccentricity squared

    inline double degToRad(double degrees) { return degrees * M_PI / 180.0; }

    /// Geodetic to earth-centred, earth-fixed.
    inline void geodeticToECEF(double lat_rad, double lon_rad, double alt_m,
                               double &x, double &y, double &z)
    {
        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);

        const double N = A / std::sqrt(1.0 - E2 * sin_lat * sin_lat);

        x = (N + alt_m) * cos_lat * cos_lon;
        y = (N + alt_m) * cos_lat * sin_lon;
        z = (N * (1.0 - E2) + alt_m) * sin_lat;
    }

    /// Latitude/longitude/altitude to metres north, east and down of a
    /// reference point.
    inline void latLonAltToNED(double lat_deg, double lon_deg, double alt_m,
                               double ref_lat_deg, double ref_lon_deg, double ref_alt_m,
                               double &north, double &east, double &down)
    {
        const double lat = degToRad(lat_deg);
        const double lon = degToRad(lon_deg);
        const double ref_lat = degToRad(ref_lat_deg);
        const double ref_lon = degToRad(ref_lon_deg);

        double x, y, z, ref_x, ref_y, ref_z;
        geodeticToECEF(lat, lon, alt_m, x, y, z);
        geodeticToECEF(ref_lat, ref_lon, ref_alt_m, ref_x, ref_y, ref_z);

        const double dx = x - ref_x;
        const double dy = y - ref_y;
        const double dz = z - ref_z;

        const double sin_lat = std::sin(ref_lat);
        const double cos_lat = std::cos(ref_lat);
        const double sin_lon = std::sin(ref_lon);
        const double cos_lon = std::cos(ref_lon);

        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
        east = -sin_lon * dx + cos_lon * dy;
        down = -cos_lat * cos_lon * dx - cos_lat * sin_lon * dy - sin_lat * dz;
    }
} // namespace mirena::GNSSConversion

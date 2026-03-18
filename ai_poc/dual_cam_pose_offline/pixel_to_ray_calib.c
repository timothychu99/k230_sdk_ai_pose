/**
 * Fisheye pixel -> normalized ray using ov5647_calibration_t (no OpenCV).
 * Matches OpenCV fisheye model: theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8).
 */
#include "pixel_to_ray_calib.h"
#include <math.h>

#define EPS 1e-10
#define MAX_ITER 5

void pixel_to_ray_fisheye(const ov5647_calibration_t *cal,
                          double pixel_x, double pixel_y,
                          double *out_dx, double *out_dy, double *out_dz)
{
    double fx = cal->camera_matrix.data[0][0];
    double fy = cal->camera_matrix.data[1][1];
    double cx = cal->camera_matrix.data[0][2];
    double cy = cal->camera_matrix.data[1][2];
    double k1 = cal->dist_coeff.data[0];
    double k2 = cal->dist_coeff.data[1];
    double k3 = cal->dist_coeff.data[2];
    double k4 = cal->dist_coeff.data[3];

    /* Normalized distorted coords */
    double x_d = (pixel_x - cx) / fx;
    double y_d = (pixel_y - cy) / fy;
    double r_d = sqrt(x_d * x_d + y_d * y_d);
    double theta_d = atan(r_d);

    /* Solve theta: theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8) */
    double theta = theta_d;
    for (int i = 0; i < MAX_ITER; i++) {
        double t2 = theta * theta;
        double t4 = t2 * t2;
        double t6 = t4 * t2;
        double t8 = t4 * t4;
        double denom = 1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8;
        if (denom < 1e-15)
            break;
        theta = theta_d / denom;
    }

    /* Undistorted normalized coords */
    double scale = (r_d > EPS) ? (theta / theta_d) : 1.0;
    double x_u = scale * x_d;
    double y_u = scale * y_d;

    /* Normalized ray (z=1) */
    double nrm = sqrt(x_u * x_u + y_u * y_u + 1.0);
    if (nrm < EPS) {
        *out_dx = 0.0;
        *out_dy = 0.0;
        *out_dz = 1.0;
        return;
    }
    *out_dx = x_u / nrm;
    *out_dy = y_u / nrm;
    *out_dz = 1.0 / nrm;
}

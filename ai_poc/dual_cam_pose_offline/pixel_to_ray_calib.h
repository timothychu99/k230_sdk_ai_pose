/**
 * Pixel to ray using ov5647_calibration_t only (no OpenCV).
 * For use by keypoint_fifo_reader.elf which is built without OpenCV.
 */
#ifndef DUAL_CAM_POSE_OFFLINE_PIXEL_TO_RAY_CALIB_H
#define DUAL_CAM_POSE_OFFLINE_PIXEL_TO_RAY_CALIB_H

#include "ov5647_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Fisheye undistort and return normalized ray direction (dx, dy, dz). */
void pixel_to_ray_fisheye(const ov5647_calibration_t *cal,
                          double pixel_x, double pixel_y,
                          double *out_dx, double *out_dy, double *out_dz);

#ifdef __cplusplus
}
#endif

#endif /* DUAL_CAM_POSE_OFFLINE_PIXEL_TO_RAY_CALIB_H */

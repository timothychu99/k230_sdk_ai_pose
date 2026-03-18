/**
 * OV5647 camera calibration (fisheye) – inlined from ov5647_matrix.yaml.
 * Use this struct instead of loading the YAML at runtime.
 */
#ifndef OV5647_CALIBRATION_H
#define OV5647_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#define OV5647_CALIB_MODEL_FISHEYE 1

/** 3x3 camera matrix (row-major). */
typedef struct ov5647_camera_matrix {
    double data[3][3];
} ov5647_camera_matrix_t;

/** Distortion coefficients (k1, k2, k3, k4 for fisheye). */
typedef struct ov5647_dist_coeff {
    double data[4];
} ov5647_dist_coeff_t;

/** Single rotation vector (3 elements). */
typedef struct ov5647_rvec {
    double data[3];
} ov5647_rvec_t;

/** Single translation vector (3 elements). */
typedef struct ov5647_tvec {
    double data[3];
} ov5647_tvec_t;

/** Number of calibration boards/views in rvecs/tvecs. */
#define OV5647_CALIB_NUM_VIEWS 18

/** Full OV5647 calibration (matches ov5647_matrix.yaml). */
typedef struct ov5647_calibration {
    ov5647_camera_matrix_t camera_matrix;
    ov5647_dist_coeff_t    dist_coeff;
    int                    model;  /* OV5647_CALIB_MODEL_FISHEYE */
    ov5647_rvec_t          rvecs[OV5647_CALIB_NUM_VIEWS];
    ov5647_tvec_t          tvecs[OV5647_CALIB_NUM_VIEWS];
} ov5647_calibration_t;

/** Single global instance filled with values from ov5647_matrix.yaml. */
extern const ov5647_calibration_t ov5647_calibration;

#ifdef __cplusplus
}
#endif

#endif /* OV5647_CALIBRATION_H */

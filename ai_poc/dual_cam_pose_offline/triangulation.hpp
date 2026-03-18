/**
 * Triangulation for multi-camera 3D position estimation (nose only).
 * Ray casting, line intersection, and midpoint in world frame.
 * C++ only – no Python/OpenCV; for use on little core.
 */
#ifndef DUAL_CAM_POSE_TRIANGULATION_HPP
#define DUAL_CAM_POSE_TRIANGULATION_HPP

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 3D vector (world or camera frame). */
typedef struct triangulation_vec3 {
    float x;
    float y;
    float z;
} triangulation_vec3_t;

/** Ray: origin + normalized direction. */
typedef struct triangulation_ray {
    triangulation_vec3_t origin;
    triangulation_vec3_t direction;
} triangulation_ray_t;

/** Result of line_intersection2: closest points and midpoint. */
typedef struct triangulation_line_result {
    triangulation_vec3_t p1;
    triangulation_vec3_t p2;
    triangulation_vec3_t midpoint;
} triangulation_line_result_t;

/** Camera extrinsics: R 3x3 row-major, cam_pos in world coords. */
typedef struct triangulation_camera_extrinsics {
    float R[9];       /* row-major 3x3: world_dir = R^T * cam_dir */
    float cam_pos[3];
} triangulation_camera_extrinsics_t;

/** Defaults matching Python (MAX_RAYGAP_CM, MAX_RAY_ANGLE_DEG). */
#define TRIANGULATION_MAX_RAYGAP_CM    20.f
#define TRIANGULATION_MAX_RAY_ANGLE_DEG 180.f

/**
 * Closest points on two 3D rays and their midpoint.
 * d1, d2 must be normalized.
 */
void triangulation_line_intersection2(
    const triangulation_vec3_t *o1, const triangulation_vec3_t *d1,
    const triangulation_vec3_t *o2, const triangulation_vec3_t *d2,
    triangulation_line_result_t *out);

/**
 * True if rays are converging (angle between directions < max_angle_deg).
 */
bool triangulation_rays_are_converging(
    const triangulation_vec3_t *d1, const triangulation_vec3_t *d2,
    float max_angle_deg);

/**
 * Raygap (distance between closest points) and 3D midpoint.
 */
void triangulation_raygap_and_midpoint(
    const triangulation_vec3_t *o1, const triangulation_vec3_t *d1,
    const triangulation_vec3_t *o2, const triangulation_vec3_t *d2,
    float *raygap_cm, triangulation_vec3_t *midpoint);

/**
 * Convert camera-frame normalized ray to world-frame ray.
 * ray_cam: (dx, dy, dz) normalized in camera frame.
 * Extrinsics: R (row-major 3x3), cam_pos; world_dir = R^T * ray_cam.
 */
void triangulation_camera_ray_to_world_ray(
    const triangulation_camera_extrinsics_t *ext,
    float dx_cam, float dy_cam, float dz_cam,
    triangulation_ray_t *world_ray);

/**
 * Fill default angled setup: cam1 at -45°, cam2 at +45° (Y rotation), baseline along X.
 * baseline_cm: distance between cameras in cm (e.g. 30).
 * Call once at init; then pass ext1, ext2 to triangulate_nose.
 */
void triangulation_default_angled_setup(
    float baseline_cm,
    triangulation_camera_extrinsics_t *ext1,
    triangulation_camera_extrinsics_t *ext2);

/**
 * Triangulate nose (keypoint index 0) from one person per camera.
 * Packets must be for camera_id 0 and 1; use first person in each.
 * Returns true if a valid midpoint was computed (raygap and convergence checks passed).
 * World convention: accept midpoint only when z <= 0 (same as Python).
 */
bool triangulation_nose_from_packets(
    const void *packet_cam0,
    const void *packet_cam1,
    const triangulation_camera_extrinsics_t *ext1,
    const triangulation_camera_extrinsics_t *ext2,
    float max_raygap_cm,
    float max_ray_angle_deg,
    triangulation_vec3_t *nose_world,
    float *out_raygap_cm);

/** After a false return from triangulation_nose_from_packets, reason for failure (e.g. "no_person", "low_score", "raygap", "z_behind", "no_converge"). */
const char *triangulation_last_failure_reason(void);

#ifdef __cplusplus
}
#endif

#endif /* DUAL_CAM_POSE_TRIANGULATION_HPP */

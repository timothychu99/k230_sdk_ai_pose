/**
 * Triangulation implementation: ray/line intersection and nose triangulation.
 * No Python/OpenCV; for little core. Optimized with constexpr and inline helpers.
 */
#include "triangulation.hpp"
#include "keypoint_fifo_protocol.h"
#include <cmath>

namespace {

/* Compile-time constants: faster compile (single definition) and constant folding. */
constexpr float kPi = 3.14159265358979323846f;
constexpr float kEps = 1e-10f;
constexpr float kRadToDeg = 180.f / kPi;
constexpr float kHalf = 0.5f;
constexpr unsigned int kNoseKeypointIndex = 0U;

/* Inline math helpers: allow inlining, no extra call overhead, smaller code. */
inline void vec3_cross(const triangulation_vec3_t *a, const triangulation_vec3_t *b,
                       triangulation_vec3_t *out)
{
    out->x = a->y * b->z - a->z * b->y;
    out->y = a->z * b->x - a->x * b->z;
    out->z = a->x * b->y - a->y * b->x;
}

inline float vec3_dot(const triangulation_vec3_t *a, const triangulation_vec3_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

inline float vec3_norm(const triangulation_vec3_t *v)
{
    return std::sqrt(vec3_dot(v, v));
}

inline void vec3_normalize(triangulation_vec3_t *v)
{
    const float n = vec3_norm(v);
    if (n > kEps)
    {
        const float inv = 1.f / n;
        v->x *= inv;
        v->y *= inv;
        v->z *= inv;
    }
}

} /* anonymous namespace */

extern "C" {

void triangulation_line_intersection2(
    const triangulation_vec3_t *o1, const triangulation_vec3_t *d1,
    const triangulation_vec3_t *o2, const triangulation_vec3_t *d2,
    triangulation_line_result_t *out)
{
    triangulation_vec3_t n, n1, n2;
    vec3_cross(d1, d2, &n);
    vec3_cross(d1, &n, &n1);
    vec3_cross(d2, &n, &n2);

    triangulation_vec3_t o2mo1 = { o2->x - o1->x, o2->y - o1->y, o2->z - o1->z };
    triangulation_vec3_t o1mo2 = { o1->x - o2->x, o1->y - o2->y, o1->z - o2->z };

    float den1 = vec3_dot(d1, &n2);
    float den2 = vec3_dot(d2, &n1);

    const float t1 = (std::fabs(den1) < kEps) ? 0.f : (vec3_dot(&o2mo1, &n2) / den1);
    const float t2 = (std::fabs(den2) < kEps) ? 0.f : (vec3_dot(&o1mo2, &n1) / den2);

    out->p1.x = o1->x + d1->x * t1;
    out->p1.y = o1->y + d1->y * t1;
    out->p1.z = o1->z + d1->z * t1;

    out->p2.x = o2->x + d2->x * t2;
    out->p2.y = o2->y + d2->y * t2;
    out->p2.z = o2->z + d2->z * t2;

    out->midpoint.x = (out->p1.x + out->p2.x) * kHalf;
    out->midpoint.y = (out->p1.y + out->p2.y) * kHalf;
    out->midpoint.z = (out->p1.z + out->p2.z) * kHalf;
}

bool triangulation_rays_are_converging(
    const triangulation_vec3_t *d1, const triangulation_vec3_t *d2,
    float max_angle_deg)
{
    float dot = vec3_dot(d1, d2);
    if (dot > 1.f)  dot = 1.f;
    if (dot < -1.f) dot = -1.f;
    const float angle_rad = std::acos(dot);
    const float angle_deg = angle_rad * kRadToDeg;
    return angle_deg < max_angle_deg;
}

void triangulation_raygap_and_midpoint(
    const triangulation_vec3_t *o1, const triangulation_vec3_t *d1,
    const triangulation_vec3_t *o2, const triangulation_vec3_t *d2,
    float *raygap_cm, triangulation_vec3_t *midpoint)
{
    triangulation_line_result_t res;
    triangulation_line_intersection2(o1, d1, o2, d2, &res);
    triangulation_vec3_t diff = {
        res.p2.x - res.p1.x,
        res.p2.y - res.p1.y,
        res.p2.z - res.p1.z
    };
    *raygap_cm = vec3_norm(&diff);
    *midpoint = res.midpoint;
}

void triangulation_camera_ray_to_world_ray(
    const triangulation_camera_extrinsics_t *ext,
    float dx_cam, float dy_cam, float dz_cam,
    triangulation_ray_t *world_ray)
{
    /* R is row-major 3x3; R^T * v = columns of R dotted with v */
    const float *R = ext->R;
    world_ray->origin.x = ext->cam_pos[0];
    world_ray->origin.y = ext->cam_pos[1];
    world_ray->origin.z = ext->cam_pos[2];
    world_ray->direction.x = R[0] * dx_cam + R[3] * dy_cam + R[6] * dz_cam;
    world_ray->direction.y = R[1] * dx_cam + R[4] * dy_cam + R[7] * dz_cam;
    world_ray->direction.z = R[2] * dx_cam + R[5] * dy_cam + R[8] * dz_cam;
    vec3_normalize(&world_ray->direction);
}

static void set_Ry(float deg, float R[9])
{
    constexpr float kDegToRad = kPi / 180.f;
    const float rad = deg * kDegToRad;
    const float c = std::cos(rad);
    const float s = std::sin(rad);
    /* Ry(θ) = [ c 0 s; 0 1 0; -s 0 c ] row-major */
    R[0] = c;  R[1] = 0.f; R[2] = s;
    R[3] = 0.f; R[4] = 1.f; R[5] = 0.f;
    R[6] = -s; R[7] = 0.f; R[8] = c;
}

void triangulation_default_angled_setup(
    float baseline_cm,
    triangulation_camera_extrinsics_t *ext1,
    triangulation_camera_extrinsics_t *ext2)
{
    const float half = baseline_cm * kHalf;
    /* Parallel cameras: both look along +Z, spaced along X. */
    set_Ry(0.f, ext1->R);
    ext1->cam_pos[0] = -half;
    ext1->cam_pos[1] = 0.f;
    ext1->cam_pos[2] = 0.f;

    set_Ry(0.f, ext2->R);
    ext2->cam_pos[0] = half;
    ext2->cam_pos[1] = 0.f;
    ext2->cam_pos[2] = 0.f;
}

static const char *s_last_failure = "ok";

const char *triangulation_last_failure_reason(void)
{
    return s_last_failure;
}

bool triangulation_nose_from_packets(
    const void *packet_cam0,
    const void *packet_cam1,
    const triangulation_camera_extrinsics_t *ext1,
    const triangulation_camera_extrinsics_t *ext2,
    float max_raygap_cm,
    float max_ray_angle_deg,
    triangulation_vec3_t *nose_world,
    float *out_raygap_cm)
{
    const auto *p0 = static_cast<const dual_cam_pose_packet_t *>(packet_cam0);
    const auto *p1 = static_cast<const dual_cam_pose_packet_t *>(packet_cam1);

    s_last_failure = "ok";
    if (p0->person_count == 0 || p1->person_count == 0)
    {
        s_last_failure = "no_person";
        return false;
    }

    constexpr float kMinScore = 1e-4f;
    const dual_cam_pose_kp_t *kp0 = &p0->persons[0].keypoints[kNoseKeypointIndex];
    const dual_cam_pose_kp_t *kp1 = &p1->persons[0].keypoints[kNoseKeypointIndex];

    if (kp0->score < kMinScore || kp1->score < kMinScore)
    {
        s_last_failure = "low_score";
        return false;
    }

    triangulation_ray_t ray0, ray1;
    triangulation_camera_ray_to_world_ray(ext1, kp0->dx, kp0->dy, kp0->dz, &ray0);
    triangulation_camera_ray_to_world_ray(ext2, kp1->dx, kp1->dy, kp1->dz, &ray1);

    if (!triangulation_rays_are_converging(&ray0.direction, &ray1.direction, max_ray_angle_deg))
    {
        s_last_failure = "no_converge";
        return false;
    }

    float raygap = 0.f;
    triangulation_vec3_t midpoint;
    triangulation_raygap_and_midpoint(
        &ray0.origin, &ray0.direction,
        &ray1.origin, &ray1.direction,
        &raygap, &midpoint);

    if (raygap > max_raygap_cm)
    {
        s_last_failure = "raygap";
        return false;
    }

    /* World Z: cameras look toward +Z, so reject points behind (z < 0). */
    if (midpoint.z < 0.f)
    {
        s_last_failure = "z_behind";
        return false;
    }

    nose_world->x = midpoint.x;
    nose_world->y = midpoint.y;
    nose_world->z = midpoint.z;
    if (out_raygap_cm)
        *out_raygap_cm = raygap;
    return true;
}

} /* extern "C" */

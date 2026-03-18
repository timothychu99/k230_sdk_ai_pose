#include "pixel_to_ray.h"
#include <opencv2/calib3d.hpp>

// -----------------------------------------------------------------------------
// Pixel to ray (calibration)
// -----------------------------------------------------------------------------

static constexpr double kRayEps = 1e-10;

/** R is 3x3 row-major; compute R^T * v (columns of R dotted with v). */
static void rt_times_vec3(const cv::Mat& R, double vx, double vy, double vz,
                          double* out_x, double* out_y, double* out_z)
{
    const double* r = R.ptr<double>();
    *out_x = r[0] * vx + r[3] * vy + r[6] * vz;
    *out_y = r[1] * vx + r[4] * vy + r[7] * vz;
    *out_z = r[2] * vx + r[5] * vy + r[8] * vz;
}

static constexpr const char kModelFisheye[] = "fisheye";

cv::Vec3d pixel_to_ray_direction(
    double pixel_x, double pixel_y,
    const cv::Mat& K, const cv::Mat& dist,
    const std::string& model)
{
    std::vector<cv::Point2f> src;
    src.reserve(1);
    src.emplace_back(static_cast<float>(pixel_x), static_cast<float>(pixel_y));
    std::vector<cv::Point2f> dst;

    if (model == kModelFisheye)
        cv::fisheye::undistortPoints(src, dst, K, dist);
    else
        cv::undistortPoints(src, dst, K, dist);

    const double nx = static_cast<double>(dst[0].x);
    const double ny = static_cast<double>(dst[0].y);
    cv::Vec3d ray_cam(nx, ny, 1.0);
    const double nrm = cv::norm(ray_cam);
    if (nrm > kRayEps)
    {
        const double inv = 1.0 / nrm;
        ray_cam[0] *= inv;
        ray_cam[1] *= inv;
        ray_cam[2] *= inv;
    }
    return ray_cam;
}

WorldRay get_world_ray(
    double pixel_x, double pixel_y,
    const cv::Mat& R, const cv::Vec3d& cam_world_pos,
    const cv::Mat& K, const cv::Mat& dist,
    const std::string& model)
{
    const cv::Vec3d ray_cam = pixel_to_ray_direction(pixel_x, pixel_y, K, dist, model);
    double dx, dy, dz;
    rt_times_vec3(R, ray_cam[0], ray_cam[1], ray_cam[2], &dx, &dy, &dz);
    cv::Vec3d ray_world_dir(dx, dy, dz);
    const double nrm = cv::norm(ray_world_dir);
    if (nrm > kRayEps)
    {
        const double inv = 1.0 / nrm;
        ray_world_dir[0] *= inv;
        ray_world_dir[1] *= inv;
        ray_world_dir[2] *= inv;
    }
    return { cam_world_pos, ray_world_dir };
}

#ifndef DUAL_CAM_POSE_OFFLINE_PIXEL_TO_RAY_H
#define DUAL_CAM_POSE_OFFLINE_PIXEL_TO_RAY_H

#include <opencv2/core.hpp>
#include <string>

/** Ray in world space: origin + normalized direction. */
struct WorldRay {
    cv::Vec3d origin;
    cv::Vec3d direction;
};

/** Undistort pixel and return normalized ray direction in camera frame. */
cv::Vec3d pixel_to_ray_direction(
    double pixel_x, double pixel_y,
    const cv::Mat& K, const cv::Mat& dist,
    const std::string& model);

/** Ray in world frame given camera rotation and position. */
WorldRay get_world_ray(
    double pixel_x, double pixel_y,
    const cv::Mat& R, const cv::Vec3d& cam_world_pos,
    const cv::Mat& K, const cv::Mat& dist,
    const std::string& model);

#endif /* DUAL_CAM_POSE_OFFLINE_PIXEL_TO_RAY_H */

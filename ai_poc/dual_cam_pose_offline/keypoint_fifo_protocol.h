#ifndef DUAL_CAM_POSE_KEYPOINT_FIFO_PROTOCOL_H
#define DUAL_CAM_POSE_KEYPOINT_FIFO_PROTOCOL_H

#include <stdint.h>

#define DUAL_CAM_POSE_KP_MAGIC 0x4B503031U
#define DUAL_CAM_POSE_KP_VERSION 1U
#define DUAL_CAM_POSE_MAX_PERSONS 16U
#define DUAL_CAM_POSE_KEYPOINT_COUNT 17U

/** Normalized ray direction (dx, dy, dz) in camera frame + score. */
typedef struct
{
    float dx;
    float dy;
    float dz;
    float score;
} dual_cam_pose_kp_t;

typedef struct
{
    float confidence;
    int32_t label;
    float box_x;
    float box_y;
    float box_w;
    float box_h;
    dual_cam_pose_kp_t keypoints[DUAL_CAM_POSE_KEYPOINT_COUNT];
} dual_cam_pose_person_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint64_t seq;
    uint64_t timestamp_us;
    uint32_t camera_id;
    uint32_t person_count;
    dual_cam_pose_person_t persons[DUAL_CAM_POSE_MAX_PERSONS];
} dual_cam_pose_packet_t;

#define DUAL_CAM_POSE_PACKET_SIZE ((uint32_t)sizeof(dual_cam_pose_packet_t))

#endif /* DUAL_CAM_POSE_KEYPOINT_FIFO_PROTOCOL_H */

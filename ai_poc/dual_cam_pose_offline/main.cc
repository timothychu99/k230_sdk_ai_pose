 /* Dual-camera offline pose detection (round-robin).
 *
 * - DEV0 (CSI0) uses sensor 37: OV_OV5647_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR_V2
 * - DEV1 (CSI1) uses sensor 38: OV_OV5647_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR
 *
 * Both devices run in VICAP_WORK_OFFLINE_MODE:
 *   sensor RAW10 -> per-device RAW input VB pool -> ISP -> CHN0 YUV420 (NV12)
 *
 * This app alternates frames between CAM0 and CAM1 and runs each through a single
 * poseDetect instance, printing how many poses were found per camera.
 *
 * Usage:
 *   ./dual_cam_pose_offline.elf <kmodel> <obj_thresh> <nms_thresh> <debug_mode>
 *
 * Example:
 *   ./dual_cam_pose_offline.elf yolov8n-pose.kmodel 0.5 0.45 0
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#include <iostream>
#include <vector>

extern "C" {
#include "k_type.h"
#include "k_vb_comm.h"
#include "k_vicap_comm.h"
#include "k_datafifo.h"
#include "mpi_vb_api.h"
#include "mpi_vicap_api.h"
#include "mpi_sys_api.h"
}

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "pose_detect.h"
#include "keypoint_fifo_protocol.h"

extern "C" {
#include "ov5647_calibration.h"
}
#include "pixel_to_ray_calib.h"

using std::cout;
using std::endl;

/* Pose model input size (matches CHN0 output) */
#define SENSOR_WIDTH   1920
#define SENSOR_HEIGHT  1080 
#define SENSOR_CHANNEL 3

/* RAW input (offline) is full sensor 1920x1080 10-bit packed to 2 bytes/pixel */
#define RAW_WIDTH   1920
#define RAW_HEIGHT  1080

/* VB layout: per-device RAW input + CHN0 YUV420 (NV12) output */
#define INPUT_BUF_NUM   4
#define OUTPUT_BUF_NUM  5

#define RAW_BUF_SIZE   VICAP_ALIGN_UP((RAW_WIDTH * RAW_HEIGHT * 2), VICAP_ALIGN_1K)
#define YUV_BUF_SIZE   VICAP_ALIGN_UP((SENSOR_WIDTH * SENSOR_HEIGHT * 3 / 2), VICAP_ALIGN_1K)

static volatile int g_quit = 0;

#define KP_FIFO_READER_INDEX 0
#define KP_FIFO_WRITER_INDEX 1
static k_datafifo_handle g_kp_fifo[2] = {
    (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE,
    (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE
};
static uint64_t g_kp_seq = 0;
static uint64_t g_kp_sent = 0;
static uint64_t g_kp_dropped = 0;
static dual_cam_pose_packet_t g_kp_packet;

static void kp_release(void *pStream)
{
    (void)pStream;
}

static uint64_t monotonic_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static int keypoint_fifo_init(void)
{
    k_datafifo_params_s writer_params = {32, DUAL_CAM_POSE_PACKET_SIZE, K_TRUE, DATAFIFO_WRITER};
    k_s32 ret = kd_datafifo_open(&g_kp_fifo[KP_FIFO_WRITER_INDEX], &writer_params);
    if (ret != K_SUCCESS)
    {
        printf("keypoint fifo open failed: 0x%x\n", ret);
        return -1;
    }

    uint64_t phy_addr = 0;
    ret = kd_datafifo_cmd(g_kp_fifo[KP_FIFO_WRITER_INDEX], DATAFIFO_CMD_GET_PHY_ADDR, &phy_addr);
    if (ret != K_SUCCESS)
    {
        printf("keypoint fifo get phy addr failed: 0x%x\n", ret);
        kd_datafifo_close(g_kp_fifo[KP_FIFO_WRITER_INDEX]);
        g_kp_fifo[KP_FIFO_WRITER_INDEX] = (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE;
        return -1;
    }

    ret = kd_datafifo_cmd(g_kp_fifo[KP_FIFO_WRITER_INDEX], DATAFIFO_CMD_SET_DATA_RELEASE_CALLBACK, (void *)kp_release);
    if (ret != K_SUCCESS)
    {
        printf("keypoint fifo set release callback failed: 0x%x\n", ret);
        kd_datafifo_close(g_kp_fifo[KP_FIFO_WRITER_INDEX]);
        g_kp_fifo[KP_FIFO_WRITER_INDEX] = (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE;
        return -1;
    }

    printf("KEYPOINT_FIFO_PHY_ADDR=0x%lx BLOCK=%u bytes\n",
           (unsigned long)phy_addr, DUAL_CAM_POSE_PACKET_SIZE);
    return 0;
}

static void keypoint_fifo_deinit(void)
{
    if (g_kp_fifo[KP_FIFO_WRITER_INDEX] != (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE)
    {
        kd_datafifo_write(g_kp_fifo[KP_FIFO_WRITER_INDEX], NULL);
        kd_datafifo_close(g_kp_fifo[KP_FIFO_WRITER_INDEX]);
        g_kp_fifo[KP_FIFO_WRITER_INDEX] = (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE;
    }
}

static void publish_keypoints_to_fifo(uint32_t camera_id, const std::vector<OutputPose> &result,
                                     const ov5647_calibration_t *cal)
{
    if (g_kp_fifo[KP_FIFO_WRITER_INDEX] == (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE)
        return;

    memset(&g_kp_packet, 0, sizeof(g_kp_packet));
    g_kp_packet.magic = DUAL_CAM_POSE_KP_MAGIC;
    g_kp_packet.version = DUAL_CAM_POSE_KP_VERSION;
    g_kp_packet.header_size = (uint16_t)(sizeof(g_kp_packet) - sizeof(g_kp_packet.persons));
    g_kp_packet.seq = ++g_kp_seq;
    g_kp_packet.timestamp_us = monotonic_time_us();
    g_kp_packet.camera_id = camera_id;
    uint32_t limit = (uint32_t)result.size();
    if (limit > DUAL_CAM_POSE_MAX_PERSONS)
        limit = DUAL_CAM_POSE_MAX_PERSONS;
    g_kp_packet.person_count = limit;

    for (uint32_t i = 0; i < limit; ++i)
    {
        const OutputPose &src = result[i];
        dual_cam_pose_person_t &dst = g_kp_packet.persons[i];
        dst.confidence = src.confidence;
        dst.label = src.label;
        dst.box_x = src.box.x;
        dst.box_y = src.box.y;
        dst.box_w = src.box.width;
        dst.box_h = src.box.height;

        size_t kp_triplets = src.kps.size() / 3;
        if (kp_triplets > DUAL_CAM_POSE_KEYPOINT_COUNT)
            kp_triplets = DUAL_CAM_POSE_KEYPOINT_COUNT;
        for (size_t k = 0; k < kp_triplets; ++k)
        {
            double px = static_cast<double>(src.kps[k * 3]);
            double py = static_cast<double>(src.kps[k * 3 + 1]);
            float score = src.kps[k * 3 + 2];
            if (score <= 0.0f)
            {
                dst.keypoints[k].dx = 0.0f;
                dst.keypoints[k].dy = 0.0f;
                dst.keypoints[k].dz = 1.0f;
                dst.keypoints[k].score = score;
                continue;
            }
            double dx, dy, dz;
            if (cal)
                pixel_to_ray_fisheye(cal, px, py, &dx, &dy, &dz);
            else
                dx = 0.0, dy = 0.0, dz = 1.0;
            dst.keypoints[k].dx = static_cast<float>(dx);
            dst.keypoints[k].dy = static_cast<float>(dy);
            dst.keypoints[k].dz = static_cast<float>(dz);
            dst.keypoints[k].score = score;
        }
    }

    k_u32 avail_write_len = 0;
    k_s32 ret = kd_datafifo_cmd(g_kp_fifo[KP_FIFO_WRITER_INDEX], DATAFIFO_CMD_GET_AVAIL_WRITE_LEN, &avail_write_len);
    if (ret != K_SUCCESS)
    {
        printf("keypoint fifo get avail write len failed: 0x%x\n", ret);
        g_kp_dropped++;
        return;
    }

    if (avail_write_len < DUAL_CAM_POSE_PACKET_SIZE)
    {
        g_kp_dropped++;
        return;
    }

    ret = kd_datafifo_write(g_kp_fifo[KP_FIFO_WRITER_INDEX], &g_kp_packet);
    if (ret != K_SUCCESS)
    {
        printf("keypoint fifo write failed: 0x%x\n", ret);
        g_kp_dropped++;
        return;
    }

    ret = kd_datafifo_cmd(g_kp_fifo[KP_FIFO_WRITER_INDEX], DATAFIFO_CMD_WRITE_DONE, NULL);
    if (ret != K_SUCCESS)
    {
        printf("keypoint fifo write done failed: 0x%x\n", ret);
        g_kp_dropped++;
        return;
    }

    g_kp_sent++;
}

static void sig_handler(int sig)
{
    (void)sig;
    g_quit = 1;
}

static void vb_init_offline(void)
{
    k_vb_config config;
    memset(&config, 0, sizeof(config));
    config.max_pool_cnt = 64;

    /* DEV0 RAW input */
    config.comm_pool[0].blk_cnt  = INPUT_BUF_NUM;
    config.comm_pool[0].mode     = VB_REMAP_MODE_NOCACHE;
    config.comm_pool[0].blk_size = RAW_BUF_SIZE;

    /* DEV0 CHN0 YUV420 output */
    config.comm_pool[1].blk_cnt  = OUTPUT_BUF_NUM;
    config.comm_pool[1].mode     = VB_REMAP_MODE_NOCACHE;
    config.comm_pool[1].blk_size = YUV_BUF_SIZE;

    /* DEV1 RAW input */
    config.comm_pool[2].blk_cnt  = INPUT_BUF_NUM;
    config.comm_pool[2].mode     = VB_REMAP_MODE_NOCACHE;
    config.comm_pool[2].blk_size = RAW_BUF_SIZE;

    /* DEV1 CHN0 YUV420 output */
    config.comm_pool[3].blk_cnt  = OUTPUT_BUF_NUM;
    config.comm_pool[3].mode     = VB_REMAP_MODE_NOCACHE;
    config.comm_pool[3].blk_size = YUV_BUF_SIZE;
 
    if (kd_mpi_vb_set_config(&config) != 0)
    {
        printf("vb_set_config failed\n");
        exit(-1);
    }

    k_vb_supplement_config sc;
    memset(&sc, 0, sizeof(sc));
    sc.supplement_config |= VB_SUPPLEMENT_JPEG_MASK;
    if (kd_mpi_vb_set_supplement_config(&sc) != 0)
    {
        printf("vb_set_supplement_config failed\n");
        exit(-1);
    }

    if (kd_mpi_vb_init() != 0)
    {
        printf("vb_init failed\n");
        exit(-1);
    }
}

/* Init one VICAP device in offline mode with:
 *   - RAW input buffer (dev_attr.buffer_*)
 *   - CHN0 YUV420 (NV12) at SENSOR_WIDTH x SENSOR_HEIGHT
 *   - mirror mode (use VICAP_MIRROR_BOTH for 180-degree rotation)
 */
static int init_vicap_offline(k_vicap_dev dev, k_vicap_sensor_type sensor_type, k_vicap_mirror mirror)
{
    k_s32 ret;
    k_vicap_sensor_info sensor_info;
    k_vicap_dev_attr dev_attr;
    k_vicap_chn_attr chn_attr;

    memset(&sensor_info, 0, sizeof(sensor_info));
    ret = kd_mpi_vicap_get_sensor_info(sensor_type, &sensor_info);
    if (ret)
    {
        printf("get_sensor_info failed for type %d\n", (int)sensor_type);
        return ret;
    }

    
    memset(&dev_attr, 0, sizeof(dev_attr));
    dev_attr.acq_win.h_start = 0;
    dev_attr.acq_win.v_start = 0;
    dev_attr.acq_win.width   = RAW_WIDTH;
    dev_attr.acq_win.height  = RAW_HEIGHT;

    dev_attr.mode        = VICAP_WORK_OFFLINE_MODE;
    dev_attr.buffer_num  = INPUT_BUF_NUM;
    dev_attr.buffer_size = RAW_BUF_SIZE;

    dev_attr.pipe_ctrl.data             = 0xFFFFFFFF;
    dev_attr.pipe_ctrl.bits.af_enable   = 0;
    dev_attr.pipe_ctrl.bits.ahdr_enable = 0;
    /* Turn on ISP 3D denoise to suppress capture grain. */
    dev_attr.pipe_ctrl.bits.dnr3_enable = 0;
    dev_attr.dw_enable                  = K_FALSE;

    dev_attr.cpature_frame = 0;
    dev_attr.mirror        = mirror;
    memcpy(&dev_attr.sensor_info, &sensor_info, sizeof(k_vicap_sensor_info));

    ret = kd_mpi_vicap_set_dev_attr(dev, dev_attr);
    if (ret)
    {
        printf("kd_mpi_vicap_set_dev_attr failed for dev %d\n", (int)dev);
        return ret;
    }

    /* CHN0: YUV420 (NV12) at inference resolution */
    memset(&chn_attr, 0, sizeof(chn_attr));
    chn_attr.out_win.h_start = 0;
    chn_attr.out_win.v_start = 0;
    chn_attr.out_win.width   = SENSOR_WIDTH;
    chn_attr.out_win.height  = SENSOR_HEIGHT;

    chn_attr.crop_win    = dev_attr.acq_win;
    chn_attr.scale_win   = chn_attr.out_win;
    chn_attr.crop_enable = K_FALSE;
    chn_attr.scale_enable = K_TRUE;
    chn_attr.chn_enable   = K_TRUE;

    chn_attr.pix_format   = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    chn_attr.buffer_num  = OUTPUT_BUF_NUM;
    chn_attr.buffer_size = YUV_BUF_SIZE;

    ret = kd_mpi_vicap_set_chn_attr(dev, VICAP_CHN_ID_0, chn_attr);
    if (ret)
    {
        printf("kd_mpi_vicap_set_chn_attr failed for dev %d chn0\n", (int)dev);
        return ret;
    }

    /* Use XML/JSON ISP database for RAW sensors */
    ret = kd_mpi_vicap_set_database_parse_mode(dev, VICAP_DATABASE_PARSE_XML_JSON);
    if (ret)
    {
        printf("kd_mpi_vicap_set_database_parse_mode failed for dev %d\n", (int)dev);
        return ret;
    }

    ret = kd_mpi_vicap_init(dev);
    if (ret)
    {
        printf("kd_mpi_vicap_init failed for dev %d\n", (int)dev);
        return ret;
    }

    return 0;
}

static void copy_bgr_to_chw_planes(const cv::Mat &bgr, void *dst, int width, int height)
{
    cv::Mat channels[3];
    cv::split(bgr, channels); /* B, G, R */

    uint8_t *out = (uint8_t *)dst;
    const size_t plane = (size_t)width * (size_t)height;
    for (int c = 0; c < 3; ++c)
    {
        uint8_t *plane_dst = out + (size_t)c * plane;
        if (channels[c].isContinuous())
        {
            memcpy(plane_dst, channels[c].data, plane);
        }
        else
        {
            for (int y = 0; y < height; ++y)
                memcpy(plane_dst + (size_t)y * width, channels[c].ptr<uint8_t>(y), (size_t)width);
        }
    }
}

/* Copy dumped YUV420 (NV12) frame to BGR CHW at dst (B plane, G plane, R plane). */
static int copy_dump_yuv_nv12_to_bgr_chw(const k_video_frame_info *dump_info, void *dst,
                                         int width, int height, const char *label)
{
    const k_u32 stride0 = dump_info->v_frame.stride[0] >= (k_u32)width ? dump_info->v_frame.stride[0] : (k_u32)width;
    const k_u32 stride1 = dump_info->v_frame.stride[1] >= (k_u32)width ? dump_info->v_frame.stride[1] : (k_u32)width;
    const size_t y_size = (size_t)stride0 * (size_t)height;
    const size_t uv_size = (size_t)stride1 * (size_t)(height / 2);

    void *y_ptr = kd_mpi_sys_mmap_cached(dump_info->v_frame.phys_addr[0], y_size);
    if (!y_ptr)
    {
        printf("%s: mmap Y plane failed\n", label);
        return -1;
    }

    cv::Mat bgr;

    /* Common path: split planes (Y + UV) */
    if (dump_info->v_frame.phys_addr[1])
    {
        void *uv_ptr = kd_mpi_sys_mmap_cached(dump_info->v_frame.phys_addr[1], uv_size);
        if (!uv_ptr)
        {
            kd_mpi_sys_munmap(y_ptr, y_size);
            printf("%s: mmap UV plane failed\n", label);
            return -1;
        }

        const cv::Mat y_mat(height, width, CV_8UC1, y_ptr, stride0);
        const cv::Mat uv_mat(height / 2, width / 2, CV_8UC2, uv_ptr, stride1);
        cv::cvtColorTwoPlane(y_mat, uv_mat, bgr, cv::COLOR_YUV2BGR_NV12);

        kd_mpi_sys_munmap(uv_ptr, uv_size);
        kd_mpi_sys_munmap(y_ptr, y_size);
    }
    else
    {
        /* Fallback: packed NV12 in one buffer (Y then UV). */
        const size_t nv12_size = (size_t)width * (size_t)height * 3 / 2;
        kd_mpi_sys_munmap(y_ptr, y_size);
        void *nv12 = kd_mpi_sys_mmap_cached(dump_info->v_frame.phys_addr[0], nv12_size);
        if (!nv12)
        {
            printf("%s: mmap NV12 failed\n", label);
            return -1;
        }

        const cv::Mat yuv_mat(height * 3 / 2, width, CV_8UC1, nv12);
        cv::cvtColor(yuv_mat, bgr, cv::COLOR_YUV2BGR_NV12);
        kd_mpi_sys_munmap(nv12, nv12_size);
    }

    copy_bgr_to_chw_planes(bgr, dst, width, height);
    return 0;
}

/* Save BGR888 planar buffer as JPEG (planar: B plane, G plane, R plane). */
static int save_bgr_planar_jpg(const uint8_t *b, const uint8_t *g, const uint8_t *r,
                               int width, int height, const char *path)
{
    cv::Mat mat(height, width, CV_8UC3);
    for (int y = 0; y < height; y++)
    {
        uint8_t *row = mat.ptr<uint8_t>(y);
        for (int x = 0; x < width; x++)
        {
            size_t i = (size_t)y * width + x;
            row[x * 3 + 0] = b[i];
            row[x * 3 + 1] = g[i];
            row[x * 3 + 2] = r[i];
        }
    }
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 95 };
    return cv::imwrite(path, mat, params) ? 0 : -1;
}

static void stop_vicap(k_vicap_dev dev)
{
    kd_mpi_vicap_stop_stream(dev);
    kd_mpi_vicap_deinit(dev);
}

/* Print one frame's pose result as JSON: {"camera":"...","count":n,"poses":[...]}.
 * Keypoints are output as [dx,dy,dz,score] (normalized ray in camera frame + score) via pixel_to_ray. */
static void print_pose_result_json(const char *cam_label,
                                   const std::vector<OutputPose> &result,
                                   const ov5647_calibration_t *cal)
{
    printf("{\"camera\":\"%s\",\"count\":%zu,\"poses\":[", cam_label, result.size());
    for (size_t i = 0; i < result.size(); i++)
    {
        const OutputPose &p = result[i];
        if (i)
            printf(",");
        printf("{\"confidence\":%.4f,\"box\":{\"x\":%.2f,\"y\":%.2f,\"w\":%.2f,\"h\":%.2f},\"label\":%d,\"keypoints\":[",
               p.confidence, p.box.x, p.box.y, p.box.width, p.box.height, p.label);
        for (size_t k = 0; k + 2 < p.kps.size(); k += 3)
        {
            double x = static_cast<double>(p.kps[k]);
            double y = static_cast<double>(p.kps[k + 1]);
            float score = p.kps[k + 2];
            double dx, dy, dz;
            if (cal)
                pixel_to_ray_fisheye(cal, x, y, &dx, &dy, &dz);
            else
                dx = dy = 0.0, dz = 1.0;
            if (k)
                printf(",");
            printf("[%.6f,%.6f,%.6f,%.4f]", dx, dy, dz, score);
        }
        printf("]}");
    }
    printf("}\n");
    fflush(stdout);
}

int main(int argc, char *argv[])
{
    if (argc < 5)
    {
        printf("Usage: %s <kmodel> <obj_thresh> <nms_thresh> <debug_mode>\n", argv[0]);
        return -1;
    }

    const char *kmodel_path = argv[1];
    float obj_thresh = atof(argv[2]);
    float nms_thresh = atof(argv[3]);
    int debug_mode = atoi(argv[4]);

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("Initializing VB for offline dual-camera...\n");
    vb_init_offline();

    if (keypoint_fifo_init() != 0)
        printf("keypoint fifo disabled; continuing without keypoint DMA output.\n");

    /* DEV0 = CSI0 sensor 37 (180-degree rotation), DEV1 = CSI1 sensor 38 */
    printf("Initializing VICAP DEV0 (sensor 37, 180-degree rotation)...\n");
    if (init_vicap_offline(VICAP_DEV_ID_0, OV_OV5647_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR, VICAP_MIRROR_VER) != 0)
    {
        printf("DEV0 init failed\n");
        keypoint_fifo_deinit();
        kd_mpi_vb_exit();
        return -1;
    }

    printf("Initializing VICAP DEV1 (sensor 38)...\n");
    if (init_vicap_offline(VICAP_DEV_ID_1, OV_OV5647_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR, VICAP_MIRROR_VER) != 0)
    {
        printf("DEV1 init failed\n");
        stop_vicap(VICAP_DEV_ID_0);
        keypoint_fifo_deinit();
        kd_mpi_vb_exit();
        return -1;
    }

    printf("Starting both VICAP streams...\n");
    if (kd_mpi_vicap_start_stream(VICAP_DEV_ID_0) != 0)
        printf("kd_mpi_vicap_start_stream DEV0 failed\n");
    if (kd_mpi_vicap_start_stream(VICAP_DEV_ID_1) != 0)
        printf("kd_mpi_vicap_start_stream DEV1 failed\n");

    /* Shared AI input buffer for poseDetect */
    size_t ai_size = (size_t)SENSOR_CHANNEL * SENSOR_HEIGHT * SENSOR_WIDTH;
    size_t paddr = 0;
    void *vaddr = nullptr;
    int ret = kd_mpi_sys_mmz_alloc_cached(&paddr, &vaddr, "dual_pose_ai", "anonymous", ai_size);
    if (ret)
    {
        printf("kd_mpi_sys_mmz_alloc_cached failed: ret=%d errno=%s\n", ret, strerror(errno));
        stop_vicap(VICAP_DEV_ID_0);
        stop_vicap(VICAP_DEV_ID_1);
        keypoint_fifo_deinit();
        kd_mpi_vb_exit();
        return -1;
    }

    FrameCHWSize isp_shape = { SENSOR_CHANNEL, SENSOR_HEIGHT, SENSOR_WIDTH };
    poseDetect pd(kmodel_path, obj_thresh, nms_thresh, isp_shape,
                  reinterpret_cast<uintptr_t>(vaddr),
                  static_cast<uintptr_t>(paddr),
                  debug_mode);

    cv::Vec4d params = pd.params;
    std::vector<OutputPose> result;
    result.reserve(DUAL_CAM_POSE_MAX_PERSONS);

    const ov5647_calibration_t *cal = &ov5647_calibration;
    printf("Calibration: fx=%.1f fy=%.1f cx=%.1f cy=%.1f model=%s\n",
           cal->camera_matrix.data[0][0], cal->camera_matrix.data[1][1],
           cal->camera_matrix.data[0][2], cal->camera_matrix.data[1][2],
           cal->model == OV5647_CALIB_MODEL_FISHEYE ? "fisheye" : "pinhole");

    k_vicap_dev devs[2] = { VICAP_DEV_ID_0, VICAP_DEV_ID_1 };
    const char *labels[2] = { "CAM0", "CAM1" };
    int turn = 0;

    printf("Dual-camera offline pose detection running (round-robin). Ctrl+C to quit.\n");

    while (!g_quit)
    {
        k_video_frame_info dump_info;
        memset(&dump_info, 0, sizeof(dump_info));

        k_vicap_dev cur_dev = devs[turn];
        const char *cam_label = labels[turn];

        /* CHN0: YUV420 (NV12); dump and convert to BGR CHW for pose */
        ret = kd_mpi_vicap_dump_frame(cur_dev, VICAP_CHN_ID_0, VICAP_DUMP_YUV, &dump_info, 1000);
        if (ret)
        {
            static uint32_t dump_fail_cnt = 0;
            if ((++dump_fail_cnt % 100U) == 1U)
                printf("%s: kd_mpi_vicap_dump_frame ch1 timeout/err=%d (cnt=%u)\n",
                       cam_label, ret, dump_fail_cnt);
            turn = 1 - turn;
            continue;
        }

        if (copy_dump_yuv_nv12_to_bgr_chw(&dump_info, vaddr, SENSOR_WIDTH, SENSOR_HEIGHT, cam_label) != 0)
        {
            kd_mpi_vicap_dump_release(cur_dev, VICAP_CHN_ID_0, &dump_info);
            turn = 1 - turn; 
            continue;
        }

        result.clear();
        pd.pre_process();
        pd.inference();
        pd.post_process(result, params);

//        print_pose_result_json(cam_label, result, cal);
        publish_keypoints_to_fifo((uint32_t)turn, result, cal);

        ret = kd_mpi_vicap_dump_release(cur_dev, VICAP_CHN_ID_0, &dump_info);
        if (ret)
        {
            printf("%s: kd_mpi_vicap_dump_release failed=%d\n", cam_label, ret);
        }

        turn = 1 - turn;
    }

    printf("Saving final VICAP frames as JPEGs...\n");
    for (int d = 0; d < 2; d++)
    {
        k_vicap_dev dev = devs[d];
        const char *label = labels[d];
        char path[64];
        snprintf(path, sizeof(path), "vicap_%s.jpg", d ? "cam1" : "cam0");

        k_video_frame_info dump_info;
        memset(&dump_info, 0, sizeof(dump_info));
        ret = kd_mpi_vicap_dump_frame(dev, VICAP_CHN_ID_0, VICAP_DUMP_YUV, &dump_info, 1000);
        if (ret)
        {
            printf("%s: dump_frame for save failed (%d), skipping %s\n", label, ret, path);
            continue;
        }

        if (copy_dump_yuv_nv12_to_bgr_chw(&dump_info, vaddr, SENSOR_WIDTH, SENSOR_HEIGHT, label) == 0)
        {
            size_t plane = (size_t)SENSOR_WIDTH * SENSOR_HEIGHT;
            const uint8_t *b = (const uint8_t *)vaddr;
            const uint8_t *g = b + plane;
            const uint8_t *r = g + plane;
            if (save_bgr_planar_jpg(b, g, r, SENSOR_WIDTH, SENSOR_HEIGHT, path) == 0)
                printf("Saved %s\n", path);
            else
                printf("%s: imwrite failed for %s\n", label, path);
        }
        else
            printf("%s: YUV to BGR copy for save failed\n", label);
        kd_mpi_vicap_dump_release(dev, VICAP_CHN_ID_0, &dump_info);
    }

    printf("Stopping VICAP and freeing resources...\n");
    stop_vicap(VICAP_DEV_ID_0);
    stop_vicap(VICAP_DEV_ID_1);
    kd_mpi_vb_exit();
    keypoint_fifo_deinit();

    ret = kd_mpi_sys_mmz_free(paddr, vaddr);
    if (ret)
    {
        printf("kd_mpi_sys_mmz_free failed: ret=%d errno=%s\n", ret, strerror(errno));
    }

    printf("dual_cam_pose_offline exit. keypoint sent=%lu dropped=%lu\n",
           (unsigned long)g_kp_sent, (unsigned long)g_kp_dropped);
    return 0;
}


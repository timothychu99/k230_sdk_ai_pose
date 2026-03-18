/**
 * Keypoint FIFO reader – reads pose packets from the data FIFO created by
 * dual_cam_pose_offline using kd_datafifo_open_by_addr and kd_datafifo_read.
 * Performs triangulation for nose (keypoint 0) when packets from both cameras are available.
 */

#include <arpa/inet.h>
#include <fcntl.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

extern "C" {
#include "k_datafifo.h"
#include "k_type.h"
}

#include "keypoint_fifo_protocol.h"
#include "triangulation.hpp"

/* Defaults matching dual_cam_pose_offline writer (override with KEYPOINT_FIFO_PHY_ADDR / argv) */
static constexpr uint64_t KEYPOINT_FIFO_DEFAULT_PHY_ADDR = 0x13d54000ULL;
static constexpr uint32_t KEYPOINT_FIFO_BLOCK_SIZE = DUAL_CAM_POSE_PACKET_SIZE;
static constexpr float TRIANGULATION_BASELINE_CM = 22.f;
/** Set to your PC LAN IP (e.g. 192.168.68.XXX). */
static constexpr const char *UDP_TARGET_IP = "192.168.1.188";
static constexpr uint16_t UDP_TARGET_PORT = 5800;

static int g_udp_sock = -1;
static struct sockaddr_in g_udp_dest;

static volatile int g_quit = 0;

/* Last packet per camera for triangulation (camera_id 0 and 1). */
static dual_cam_pose_packet_t g_last_cam0;
static dual_cam_pose_packet_t g_last_cam1;
static int g_has_cam0 = 0;
static int g_has_cam1 = 0;

static triangulation_camera_extrinsics_t g_ext1;
static triangulation_camera_extrinsics_t g_ext2;

static void sig_handler(int sig)
{
    (void)sig;
    g_quit = 1;
}

static int setup_udp_socket(void)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0)
        return -1;
    int fl = fcntl(s, F_GETFL, 0);
    if (fl < 0 || fcntl(s, F_SETFL, fl | O_NONBLOCK) < 0)
    {
        close(s);
        return -1;
    }
    memset(&g_udp_dest, 0, sizeof(g_udp_dest));
    g_udp_dest.sin_family = AF_INET;
    g_udp_dest.sin_port = htons(UDP_TARGET_PORT);
    if (inet_pton(AF_INET, UDP_TARGET_IP, &g_udp_dest.sin_addr) != 1)
    {
        close(s);
        return -1;
    }
    return s;
}

static void run_nose_triangulation(void)
{
    if (!g_has_cam0 || !g_has_cam1)
        return;
    triangulation_vec3_t nose_world;
    float raygap_cm = 0.f;
    bool ok = triangulation_nose_from_packets(
        &g_last_cam0, &g_last_cam1,
        &g_ext1, &g_ext2,
        TRIANGULATION_MAX_RAYGAP_CM,
        TRIANGULATION_MAX_RAY_ANGLE_DEG,
        &nose_world,
        &raygap_cm);

    if (!ok || g_udp_sock < 0)
        return;
    /* Local floats then CSV over UDP (non-blocking; drop on EAGAIN). */
    const float x = nose_world.x;
    const float y = nose_world.y;
    const float z = nose_world.z;
    (void)raygap_cm;
    char buf[96];
    int n = snprintf(buf, sizeof(buf), "x: %.6f, y: %.6f, z: %.6f \n", x, y, z);
    if (n <= 0 || n >= (int)sizeof(buf))
        return;
    (void)sendto(g_udp_sock, buf, (size_t)n, 0,
                 reinterpret_cast<struct sockaddr *>(&g_udp_dest),
                 sizeof(g_udp_dest));
    /* Non-blocking: EAGAIN/EWOULDBLOCK = drop frame; no logging in hot path. */
}

int main(int argc, char **argv)
{  
    uint64_t phy_addr = KEYPOINT_FIFO_DEFAULT_PHY_ADDR;
    if (argc >= 2)
    {
        if (sscanf(argv[1], "%" SCNx64, &phy_addr) != 1)
        {
            fprintf(stderr, "Usage: %s [phy_addr_hex]\n", argv[0]);
            fprintf(stderr, "  default: KEYPOINT_FIFO_PHY_ADDR=0x%lx BLOCK=%u\n",
                    (unsigned long)KEYPOINT_FIFO_DEFAULT_PHY_ADDR, KEYPOINT_FIFO_BLOCK_SIZE);
            return 1;
        }
    }
    else
    {
        const char *env = getenv("KEYPOINT_FIFO_PHY_ADDR");
        if (env && sscanf(env, "%" SCNx64, &phy_addr) == 1)
            /* use env value */;
        /* else keep KEYPOINT_FIFO_DEFAULT_PHY_ADDR */
    }

    if (phy_addr == 0)
    {
        fprintf(stderr, "invalid phy_addr=0\n");
        return 1;
    }

    signal(SIGINT, sig_handler);

    k_datafifo_handle handle = (k_datafifo_handle)K_DATAFIFO_INVALID_HANDLE;
    k_datafifo_params_s params = {32, KEYPOINT_FIFO_BLOCK_SIZE, K_TRUE, DATAFIFO_READER};

    k_s32 ret = kd_datafifo_open_by_addr(&handle, &params, phy_addr);
    if (ret != K_SUCCESS)
    {
        fprintf(stderr, "kd_datafifo_open_by_addr failed: 0x%x (addr=0x%lx)\n",
                (unsigned)ret, (unsigned long)phy_addr);
        return 1;
    }

    g_udp_sock = setup_udp_socket();
    if (g_udp_sock < 0)
    {
        fprintf(stderr, "UDP socket setup failed (target %s:%u)\n",
                UDP_TARGET_IP, (unsigned)UDP_TARGET_PORT);
        kd_datafifo_close(handle);
        return 1;
    }

    triangulation_default_angled_setup(TRIANGULATION_BASELINE_CM, &g_ext1, &g_ext2);

    while (!g_quit)
    {
        k_u32 avail = 0;
        ret = kd_datafifo_cmd(handle, DATAFIFO_CMD_GET_AVAIL_READ_LEN, &avail);
        if (ret != K_SUCCESS)
        {
            fprintf(stderr, "GET_AVAIL_READ_LEN failed: 0x%x\n", (unsigned)ret);
            break;
        }

        if (avail > 0 && avail <= KEYPOINT_FIFO_BLOCK_SIZE * 32)
        {
            void *p_data = NULL;
            ret = kd_datafifo_read(handle, &p_data);
            if (ret != K_SUCCESS)
            {
                fprintf(stderr, "kd_datafifo_read failed: 0x%x\n", (unsigned)ret);
                break;
            }

            if (p_data)
            {
                /* Copy packet out of hardware buffer, then READ_DONE immediately. */
                static dual_cam_pose_packet_t packet_copy;
                memcpy(&packet_copy, p_data, DUAL_CAM_POSE_PACKET_SIZE);
                ret = kd_datafifo_cmd(handle, DATAFIFO_CMD_READ_DONE, p_data);
                if (ret != K_SUCCESS)
                {
                    fprintf(stderr, "READ_DONE failed: 0x%x\n", (unsigned)ret);
                    break;
                }
                p_data = NULL;

                const dual_cam_pose_packet_t *pkt = &packet_copy;
                if (pkt->magic == DUAL_CAM_POSE_KP_MAGIC)
                {
                    if (pkt->camera_id == 0)
                    {
                        memcpy(&g_last_cam0, pkt, DUAL_CAM_POSE_PACKET_SIZE);
                        g_has_cam0 = 1;
                    }
                    else if (pkt->camera_id == 1)
                    {
                        memcpy(&g_last_cam1, pkt, DUAL_CAM_POSE_PACKET_SIZE);
                        g_has_cam1 = 1;
                        run_nose_triangulation();
                    }
                }
            }
        }
        else
        {
            usleep(5000);
        }
    }

    if (g_udp_sock >= 0)
        close(g_udp_sock);
    kd_datafifo_close(handle);
    return 0;
}
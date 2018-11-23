/*****************************************************************//**
 *       @file  sample_save_replay.c
 *      @brief  Save replay file during DMCAM capturing 
 *
 *  Detail Decsription starts here
 *
 *   @internal
 *     Project  $Project$
 *     Created  3/24/2017 
 *    Revision  $Id$
 *     Company  Data Miracle, Shanghai
 *   Copyright  (C) 2017 Data Miracle Intelligent Technologies
 *    
 *    THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *    KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *    PARTICULAR PURPOSE.
 *
 * *******************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#include "pthread.h"
#include "dmcam.h"

#define MAX_FRAME_SIZE (320 * 240 * 4 * 2)

static dmcam_dev_t *dev;
static dmcam_dev_t dev_list[4];

static int list_all_devices(void)
{
    int dev_cnt, i;
    char uri_str[64];

    /* list all dmcam devices (max = 4) */
    dev_cnt = dmcam_dev_list(dev_list, sizeof(dev_list) / sizeof(dev_list[0]));
    printf(" %d dmcam device found\n", dev_cnt);

    /* print device URI of all detected devices */
    for (i = 0; i < dev_cnt; i++) {
        printf("  [%02u] %s\n", i, dmcam_dev_get_uri(&dev_list[i], uri_str, sizeof(uri_str)));
    }
    return dev_cnt;
}

int main(int argc, char **argv)
{
    char *fname_replay = "sample_replay.oni";
    char uri_str[64];

    if (argc > 1) {
        fname_replay = argv[1];
    }
    printf(" -> replay file set to %s\n", fname_replay);

    /* init dmcam driver */
    dmcam_init(NULL);

    /* config the log */
    dmcam_log_cfg(LOG_LEVEL_INFO, LOG_LEVEL_DEBUG, LOG_LEVEL_NONE);

    /* list devices, if no device found, exit the program  */
    if (list_all_devices() <= 0)
        goto FINAL;

    /* main section to use the device */
    {
        /* - open device - */

        // option1: open with specified device got by dmcam_dev_list
        //dev = dmcam_dev_open(&dev_list[0]);

        // option2: open with NULL to get the first detected device
        //dev = dmcam_dev_open(NULL);

        // option3: open using device URI string
        dev = dmcam_dev_open_by_uri(dmcam_dev_get_uri(&dev_list[0], uri_str, sizeof(uri_str)));
        if (!dev) {
            printf(" open device failed\n");
            goto FINAL;
        }

        /* - set capture config --*/
        dmcam_cap_cfg_t cap_cfg = {
            .cache_frames_cnt = 4, /* 4 frames can be cached in frame buffer*/
            .on_error = NULL,      /* No error callback */
            .on_frame_ready = NULL, /* No frame ready callback*/
            .en_save_replay = true, /* enable save raw data stream to replay file */
            .en_save_dist_u16 = false, /* disable save dist stream into replay file */
            .en_save_gray_u16 = false, /* disable save gray stream into replay file*/
            .fname_replay = fname_replay, /* replay filename */
        };

        dmcam_cap_config_set(dev, &cap_cfg);

        /* - start capture -*/
        dmcam_cap_start(dev);

        {
            int i, r, w, h, n_frames = 100;
            uint8_t *fbuf = malloc(MAX_FRAME_SIZE);
            dmcam_frame_t frinfo;

            for (i = 0; i < n_frames; i++) {
                /* Get 1 frame from device. 
                 * - dmcam_cap_get_frames is blocking wait all frames are ready,
                 * - dmcam_cap_get_frame is the non-blocking version*/
                r = dmcam_cap_get_frames(dev, 1, fbuf, MAX_FRAME_SIZE, &frinfo);
                if (r <= 0) {
                    printf("capture frame failed: r = %d\n", r);
                    break;
                }
                w = frinfo.frame_info.width, h = frinfo.frame_info.height;
                printf("Get frame #%d: [%ux%u, idx=%u, fmt=%u, sz=%u]\n", i, w, h, frinfo.frame_info.frame_idx,
                       frinfo.frame_info.frame_format, frinfo.frame_info.frame_size);

                {
                    uint16_t *buf_u16 = malloc(sizeof(uint16_t) * w * h);
                    /* decode one frame to gray
                     * -> trig to save gray into replay */
                    int buf_len = dmcam_frame_get_gray_u16(dev, buf_u16, w * h, fbuf, frinfo.frame_info.frame_size, &frinfo.frame_info);

                    /* decode one frame to depth
                     * -> trig to save depth into replay */
                    buf_len = dmcam_frame_get_dist_u16(dev, buf_u16, w * h, fbuf, frinfo.frame_info.frame_size, &frinfo.frame_info);

                    /* process  data */
                    (void)buf_len; 

                    free(buf_u16);
                }
            }
            free(fbuf);
        }

        /* stop capturing */
        dmcam_cap_stop(dev);

        /* close device */
        dmcam_dev_close(dev);
    }

FINAL:
    /* uninit dmcam driver */
    dmcam_uninit();
    return 0;
}

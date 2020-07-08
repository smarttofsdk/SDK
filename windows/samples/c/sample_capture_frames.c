/*****************************************************************//**
 *       @file  sample_capture_frames.c
 *      @brief  Brief Decsription
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
#include <inttypes.h>
#include <stdbool.h>
#include <time.h>
#include <assert.h>

#include "dmcam.h"
#ifdef _WIN32
    #include <WinSock.h>  // struct timeval
static int gettimeofday(struct timeval* tp, void* tzp)
{
    time_t clock;
    struct tm tm;
    SYSTEMTIME wtm;

    GetLocalTime(&wtm);
    tm.tm_year = wtm.wYear - 1900;
    tm.tm_mon = wtm.wMonth - 1;
    tm.tm_mday = wtm.wDay;
    tm.tm_hour = wtm.wHour;
    tm.tm_min = wtm.wMinute;
    tm.tm_sec = wtm.wSecond;
    tm.tm_isdst = -1;
    clock = mktime(&tm);
    tp->tv_sec = (long)clock;
    tp->tv_usec = wtm.wMilliseconds * 1000;
    return (0);
}

static struct tm* localtime_r(const time_t* timep, struct tm* result)
{
    return localtime_s(result, timep) ? NULL : result;
}
#else
    #include <sys/time.h>
#endif

#define FRAME_SIZE (640*480*4 * 2)
#define FRAME_BUF_FCNT 10

static dmcam_dev_t *dev;

int main(int argc, char **argv)
{
    int debug_level = 0;
    char *dev_uri = NULL;

    dmcam_dev_t dev_list[4];

    if (argc > 1) {
        //debug_level = atoi(argv[1]);
        //printf(" debug level set to %d\n", debug_level);
        dev_uri = argv[1];
    }

    /* init dmcam driver */
    dmcam_init(NULL); // use default log dmcam_yyyymmdd.log
    //dmcam_init("");   // use no log

    /* config the log */
    dmcam_log_cfg(LOG_LEVEL_INFO, LOG_LEVEL_TRACE, LOG_LEVEL_NONE - debug_level);

    /* main section to use the device */
    {
        if (dev_uri) {
            dev = dmcam_dev_open_by_uri(dev_uri);
        } else {
            int dev_cnt;

            /* list all dmcam devices (max = 4) */
            dev_cnt = dmcam_dev_list(dev_list, 4);

            printf(" %d dmcam device found\n", dev_cnt);
            if (dev_cnt == 0)
                goto FINAL;

            /* open the first present devic */
            /*   you can also use NULL for dmcam_dev_open to open the
             *   first device e.g. dev =  dmcam_dev_open(NULL); */
            dev = dmcam_dev_open(NULL);
            if (dev) {
                dmcam_dev_close(dev);
            }
            dev = dmcam_dev_open(&dev_list[0]);
        }
        if (!dev) {
            printf(" open device failed\n");
            goto FINAL;
        }
        /* set capture config */
        dmcam_cap_cfg_t cap_cfg = {
            .cache_frames_cnt = FRAME_BUF_FCNT, /* FRAME_BUF_FCNT frames can be cached in frame buffer*/
            .on_error = NULL,      /* No error callback */
            .on_frame_ready = NULL, /* No frame ready callback*/
            .en_save_replay = false, /* false save raw data stream to replay file */
            .en_save_dist_u16 = false, /* disable save dist stream into replay file */
            .en_save_gray_u16 = false, /* disable save gray stream into replay file*/
            .fname_replay = NULL, /* replay filename */
        };
        {
            int fps = 20;
            /* set fps & intg  */
            dmcam_param_item_t wparam[2];

            memset(wparam, 0, sizeof(wparam));

            wparam[0].param_id = PARAM_FRAME_RATE;
            wparam[0].param_val.frame_rate.fps = fps;

            wparam[1].param_id = PARAM_INTG_TIME;
            wparam[1].param_val.intg.intg_us = 1000;

            /* set param : fps / intg */
            dmcam_param_batch_set(dev, wparam, sizeof(wparam) / sizeof(wparam[0]));

            printf("set fps to %d\n", fps);
        }

        dmcam_cap_config_set(dev, &cap_cfg);

        {
            dmcam_frame_t frinfo;
            int n;
            int total_fr = 0;
            int fr_cnt;
            struct timeval ts0, ts1;
            uint8_t *fbuf = malloc(FRAME_SIZE);
            float *dist_f32 = malloc(FRAME_SIZE);
            uint16_t *gray_u16 = malloc(FRAME_SIZE);

            float total_ms = 0;

            assert(fbuf);

            dmcam_filter_disable(dev, DMCAM_FILTER_ID_MEDIAN);
            dmcam_filter_disable(dev, DMCAM_FILTER_ID_PIXEL_CALIB);

            /* start capturing frames */
            dmcam_cap_start(dev);

            gettimeofday(&ts0, NULL);
            for (n = 0; n < 30; n++) {
                int w, h;
                struct tm *fr_tm, fr_tm_s;
                struct timeval cur_tv;
                time_t fr_t;
                static dmcam_frame_info_t last_frinfo;
                int int_ms = -1;

                /* get 1 frame with blocking wait. if fps=20,
                 * dmcam_cap_get_frames will return in about 50ms */
                fr_cnt = dmcam_cap_get_frames(dev, 1, fbuf, FRAME_SIZE, &frinfo);
                if (fr_cnt < 1) { // less frames means sampling is stopped.
                    printf("capturing is stopped due to some error: %d\n", fr_cnt);
                    break;
                } 
                total_fr += fr_cnt;

                w = frinfo.frame_info.width;
                h = frinfo.frame_info.height;

                /* convert frame timestamp from unix timestamp to struct tm
                 * NOTE: localtime is not re-entrent, you should take care */
                fr_t = (time_t)frinfo.frame_info.rx_ts;
                fr_tm = localtime_r(&fr_t, &fr_tm_s);

                gettimeofday(&cur_tv, NULL);

                if (last_frinfo.rx_ts > 0) {
                    int_ms = 1000 * ((int)frinfo.frame_info.rx_ts - (int)last_frinfo.rx_ts) +
                             ((int)frinfo.frame_info.rx_us - (int)last_frinfo.rx_us) / 1000;
                }

                memcpy(&last_frinfo, &frinfo.frame_info, sizeof(last_frinfo));
                printf("get %d frames: [%u, %ux%u, %u, %u, %04u/%02u/%02u %02u:%02u:%02u.%03u][%d ms][%u/%03u]\n",
                       fr_cnt, frinfo.frame_info.frame_idx, w, h, frinfo.frame_info.frame_format,
                       frinfo.frame_info.rx_ts,
                       fr_tm->tm_year + 1900, fr_tm->tm_mon + 1, fr_tm->tm_mday,
                       fr_tm->tm_hour, fr_tm->tm_min, fr_tm->tm_sec, frinfo.frame_info.rx_us / 1000,
                       int_ms,
                       (int)cur_tv.tv_sec, (int)cur_tv.tv_usec / 1000
                      );

                //usleep(100000);
                /* decode one frame to distance */
                {
                    int dist_len = w * h;
                    float *dist = dist_f32;
                    //struct timeval dist_ts0, dist_ts1;
                    //gettimeofday(&dist_ts0, NULL);
                    int calc_len = dmcam_frame_get_dist_f32(dev, dist, dist_len, fbuf, frinfo.frame_info.frame_size, &frinfo.frame_info);

                    //gettimeofday(&dist_ts1, NULL);
                    if (calc_len == 0) {
                        printf("the frame is not valid !\n");
                    }
                    //printf("dist: spend %d ms\n", (dist_ts1.tv_sec - dist_ts0.tv_sec) * 1000 + (dist_ts1.tv_usec - dist_ts0.tv_usec) / 1000);
                    (void)calc_len;


                    /* decode one frame to pcl */
                    //{
                    //    int pcl_len = w * h * 4;
                    //    float *pcl = malloc(sizeof(float) * pcl_len);
                    //    int calc_len = dmcam_frame_get_pcl_xyzd(dev, pcl, pcl_len, dist, dist_len, w, h, true, NULL);
                    //
                    //    /* process distance data */
                    //    printf("proc pcl data ....\n");
                    //    (void)calc_len;
                    //
                    //    free(pcl);
                    //}
                    //free(dist);
                }
                /* decode one frame to gray */
                {
                    int gray_len = w * h;
                    int calc_len = dmcam_frame_get_gray_u16(dev, gray_u16, gray_len, fbuf, frinfo.frame_info.frame_size, &frinfo.frame_info);

                    assert(calc_len == gray_len);
                    //printf("proc gray data ....\n");

                    /* process gray data */
                    (void)calc_len;

                }


            }

            gettimeofday(&ts1, NULL);

            total_ms += (ts1.tv_sec - ts0.tv_sec) * 1000 + (ts1.tv_usec - ts0.tv_usec) / 1000;
            printf("Total %d frames, decode spend %d ms. fps=%.1f\n", total_fr, (int)total_ms, total_fr * 1000.0 / (float)total_ms);

            /* stop capturing */
            dmcam_cap_stop(dev);


            free(fbuf);
            free(dist_f32);
            free(gray_u16);
        }

        /* close device */
        dmcam_dev_close(dev);
    }
FINAL:

    /* uninit dmcam driver */
    dmcam_uninit();

    return 0;
}


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
#include <sys/types.h>
#include <getopt.h>
#include <stdbool.h>
#include <unistd.h>
#include <assert.h>

#include "pthread.h"
#include "dmcam.h"

#ifdef _WIN32
    #include <windows.h>
    #define sleep(s) Sleep(s*1000)
#else
#endif

#define FRAME_SIZE (320*240*4 * 2)
#define FRAME_BUF_FCNT 16

static dmcam_dev_t *dev;
#ifdef _MSC_VER
    #pragma comment(lib, "pthreadVC2.lib")
#endif
static void on_frame_rdy(dmcam_dev_t *dev, dmcam_frame_t *f)
{
    printf("cap: idx=%d, size=%d\n", f->frame_info.frame_idx, f->frame_info.frame_size);
    //usleep(300000);
}

/* return false will stop the capturing process */
//static bool on_cap_err(dmcam_dev_t *dev, int err, void *err_arg)
//{
//    printf("cap error found : %s, arg=%p\n", dmcam_error_name(err), err_arg);
//    switch (err) {
//        case DMCAM_ERR_CAP_FRAME_DISCARD:
//            printf("  data process too slow: total missing %d frames\n", (int)(size_t)err_arg);
//            break;
//        case DMCAM_ERR_CAP_STALL:
//            printf(" usb pipe stall!\n");
//            break;
//        default:
//            break;
//    }
//    return true;
//}
static pthread_t test_th;
static void* th_test_entry(void *arg)
{
    uint32_t after_msec = (size_t)arg;

    printf(" stop after %d sec\n", after_msec);
    sleep(after_msec);

    dmcam_cap_stop(dev);

    printf(" exit async stop\n");
    return NULL;
}

static void async_stop(uint32_t after_msec)
{
    if (pthread_create(&test_th, NULL, th_test_entry, (void *)(size_t)after_msec) < 0) {
        printf("create test thread failed\n");
    }
}

static void async_wait(void)
{
    pthread_join(test_th, NULL);
}

void test_async_stop(void)
{
    int i;

    printf("---- test async stop ---\n");
    for (i = 0; i < 1; i++) {
        int n;

        dmcam_cap_start(dev);
        async_stop(2);
        for (n = 0; n < 10; n++) {
            int fr_cnt = dmcam_cap_get_frames(dev, 45, NULL, 0, NULL);

            printf("get %d frames\n", fr_cnt);
            if (fr_cnt < 45) {
                printf("get frame stopped!\n");
                break;
            }
        }
        dmcam_cap_wait(dev, 10000);
        async_wait();
        sleep(1);
    }
}
/**
 * this programming model is deprecated.
 * 
 */

void test_model_deprecated(void)
{
    dmcam_cap_set_callback_on_frame_ready(dev, on_frame_rdy);

    {
        printf("---- test paradigm: start/wait/stop  ---\n");
        fprintf(stderr, " start...\n");
        assert(dmcam_cap_start(dev));
        dmcam_cap_wait(dev, 10000);
        //dmcam_cap_wait(dev, 0);
        fprintf(stderr, "stop...\n");
        assert(dmcam_cap_stop(dev));
    }
}

/**
 * this is the recommended programming model. with full 
 * parameter usage. 
 */
void test_model_new(void)
{
    dmcam_frame_t fbuf_info;
    uint8_t *fbuf = malloc(FRAME_SIZE * 20);
    int n;
    int total_fr = 0;
    int fr_cnt;

    assert(fbuf);
    printf("---- test paradigm: start/get_frames/stop  ---\n");
    dmcam_cap_set_callback_on_frame_ready(dev, NULL); // optional: disable frame ready callback
    dmcam_cap_start(dev);
    for (n = 0; n < 10; n++) {
        /* skip 20 frames */
        printf("skip 20 frames..\n");
        if (dmcam_cap_get_frames(dev, 20, NULL, 0, NULL) < 20) {
            printf("capturing is stopped due to some error!\n");
            break;
        }
        /* get 20 frames */
        fr_cnt = dmcam_cap_get_frames(dev, 20, fbuf, FRAME_SIZE * 20, &fbuf_info);
        total_fr += fr_cnt;
        printf("get %d frames: [%u, %ux%u, %u]\n",
               fr_cnt, fbuf_info.frame_info.frame_idx, 
               fbuf_info.frame_info.width, fbuf_info.frame_info.height, fbuf_info.frame_info.frame_format);
        if (fr_cnt < 20) { // less frames means sampling is stopped.
            printf("capturing is stopped due to some error!\n");
            break;
        }

        if (total_fr > 100) {
            printf("get enough frames, we stop\n");
            break;
        }
        // do some work to process every frame in fbuf
        printf("proc frames ....\n");

        /* decode one frame to distance */
        {
            int dist_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
            float *dist = malloc(sizeof(float) * dist_len);
            int calc_len = dmcam_frame_get_distance(dev, dist, dist_len, fbuf, fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);
            assert(calc_len == dist_len);

            free(dist);
        }
        /* decode one frame to gray */
        {
            int gray_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
            float *gray = malloc(sizeof(float) * gray_len);
            int calc_len = dmcam_frame_get_gray(dev, gray, gray_len, fbuf, fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);
            assert(calc_len == gray_len);

            free(gray);
        }

        // usleep(400 * 1000); // emu that proc spends time
    }
    free(fbuf);
    dmcam_cap_stop(dev);
}

void test_model_new_snapshot(void)
{
    uint8_t *fbuf = malloc(FRAME_SIZE);

    assert(fbuf);
    printf("---- test paradigm: snapshot  ---\n");
    dmcam_cap_set_callback_on_frame_ready(dev, NULL); // optional: disable frame ready callback
    printf(" * snapshot in stop status \n");
    dmcam_cap_stop(dev);
    if (!dmcam_cap_snapshot(dev, fbuf, FRAME_SIZE, NULL)) {
        printf("snapshot failed!\n");
    }
    printf("get frame @ %p\n", fbuf);

    //printf(" * snapshot in start status \n");
    {
        //dmcam_frame_t fbuf_info;
        //dmcam_cap_start(dev);
        //if (!dmcam_cap_snapshot(dev, fbuf, FRAME_SIZE, &fbuf_info)) {
        //    printf("snapshot failed!\n");
        //}
        //printf("get frame @ %p, fr_cnt=%u\n", fbuf, fbuf_info.frame_count);
        //usleep(300 * 1000);
        //if (!dmcam_cap_snapshot(dev, fbuf, FRAME_SIZE, &fbuf_info)) {
        //    printf("snapshot failed!\n");
        //}
        //printf("get frame @ %p, fr_cnt=%u\n", fbuf, fbuf_info.frame_count);
        //dmcam_cap_stop(dev);
    }
    free(fbuf);
}

int main(int argc, char **argv)
{
    int debug_level = 0;

    if (argc > 1) {
        debug_level = atoi(argv[1]);
        printf(" debug level set to %d\n", debug_level);
    }
    dmcam_init(NULL);

    dmcam_log_cfg(LOG_LEVEL_INFO, LOG_LEVEL_TRACE, LOG_LEVEL_NONE - debug_level);

#if 1
    {
        int dev_cnt;
        dmcam_dev_t dev_list[4];

        dev_cnt = dmcam_dev_list(dev_list, 4);

        printf(" %d dmcam device found\n", dev_cnt);
        if (dev_cnt == 0)
            goto FINAL;
        /* open device  */
        dev = dmcam_dev_open(&dev_list[0]);
        if (!dev) {
            printf(" open device failed\n");
            goto FINAL;
        }
        /* reset device */

        //dmcam_dev_reset(dev, DEV_RST_TFC);
        /* close device */
        dmcam_dev_close(dev);
    }
#endif
    /* open device */
    dev = dmcam_dev_open(NULL);
    if (!dev) {
        printf(" open device failed\n");
        goto FINAL;
    }
    /* set illumination power*/
    //{
    //    dmcam_param_item_t wparam;
    //
    //    memset(&wparam, 0, sizeof(wparam));
    //    wparam.param_id = PARAM_INTG_TIME;
    //    wparam.param_val_len = 1;
    //    wparam.param_val.intg.intg_us = intg_tim;
    //    assert(dmcam_param_batch_set(dev, &wparam, 1));
    //}
    /* capture frames using interval alloced buffer */
    dmcam_cap_set_frame_buffer(dev, NULL, FRAME_SIZE * FRAME_BUF_FCNT);
    /* set error callback for capturing */
    dmcam_cap_set_callback_on_error(dev, NULL);

    /* reset the usbif will help to fix stall problem */
    //dmcam_dev_reset(dev, DEV_RST_USB);

    test_model_deprecated();
    test_model_new();
    //test_model_new_snapshot();
    //
    //test_async_stop();

    dmcam_dev_close(dev);
FINAL:
    dmcam_uninit();
    return 0;
}

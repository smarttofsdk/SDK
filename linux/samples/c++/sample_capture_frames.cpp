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
#include <assert.h>

#include "dmcam.h"

#define FRAME_SIZE (320*240*4 * 2)
#define FRAME_BUF_FCNT 10

static dmcam_dev_t *dev;

int main(int argc, char **argv)
{
    int debug_level = 0;

    if (argc > 1) {
        debug_level = atoi(argv[1]);
        printf(" debug level set to %d\n", debug_level);
    }

    /* init dmcam driver */
    dmcam_init(NULL);

    /* config the log */
    dmcam_log_cfg(LOG_LEVEL_INFO, LOG_LEVEL_TRACE,dmcam_log_level_e(LOG_LEVEL_NONE - debug_level));

    /* main section to use the device */
    {
        int dev_cnt;
        dmcam_dev_t dev_list[4];

        /* list all dmcam devices (max = 4) */
        dev_cnt = dmcam_dev_list(dev_list, 4);

        printf(" %d dmcam device found\n", dev_cnt);
        if (dev_cnt == 0)
            goto FINAL;

        /* open the first present devic */
        /*   you can also use NULL for dmcam_dev_open to open the
         *   first device e.g. dev =  dmcam_dev_open(NULL); */
        dev = dmcam_dev_open(&dev_list[0]);
        if (!dev) {
            printf(" open device failed\n");
            goto FINAL;
        }

		        /* set capture config */
        // dmcam_cap_cfg_t cap_cfg = {
            // .cache_frames_cnt = FRAME_BUF_FCNT, /* FRAME_BUF_FCNT frames can be cached in frame buffer*/
            // .on_error = NULL,      /* No error callback */
            // .on_frame_ready = NULL, /* No frame ready callback*/
            // .en_save_replay = false, /* false save raw data stream to replay file */
            // .en_save_dist_u16 = false, /* disable save dist stream into replay file */
            // .en_save_gray_u16 = false, /* disable save gray stream into replay file*/
            // .fname_replay = NULL, /* replay filename */
        // };
		
	    dmcam_cap_cfg_t cap_cfg = {
            FRAME_BUF_FCNT, /* FRAME_BUF_FCNT frames can be cached in frame buffer*/
            NULL,      /* No error callback */
            NULL, /* No frame ready callback*/
            false, /* false save raw data stream to replay file */
            false, /* disable save dist stream into replay file */
            false, /* disable save gray stream into replay file*/
            NULL, /* replay filename */
        };
		
		dmcam_cap_config_set(dev,&cap_cfg);

        {
            dmcam_frame_t fbuf_info;
            int n;
            int total_fr = 0;
            int fr_cnt;
            int rd_fr_once = 10;

			uint8_t *fbuf = new uint8_t[FRAME_SIZE*rd_fr_once];

            assert(fbuf);

            /* start capturing frames */
            dmcam_cap_start(dev);

            for (n = 0; n < 10; n++) {
                int w, h;

                /* get 20 frames */
                fr_cnt = dmcam_cap_get_frames(dev, rd_fr_once, fbuf, FRAME_SIZE * rd_fr_once, &fbuf_info);
                if (fr_cnt < rd_fr_once) { // less frames means sampling is stopped.
                    printf("capturing is stopped due to some error!\n");
                    break;
                }
                total_fr += fr_cnt;
                printf("get %d frames: [%u, %ux%u, %u]\n",
                       fr_cnt, fbuf_info.frame_info.frame_idx,
                       fbuf_info.frame_info.width, fbuf_info.frame_info.height, fbuf_info.frame_info.frame_format);


                /* stop capture if get enough frames */
                if (total_fr > 100) {
                    printf("get enough frames, we stop\n");
                    break;
                }


                /* proc frames in fbuf */
                printf("proc frames ....\n"); 

                w = fbuf_info.frame_info.width;
                h = fbuf_info.frame_info.height;

                /* decode one frame to distance */
                {
                    int dist_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
					float *dist = new float[dist_len];
                    int calc_len = dmcam_frame_get_distance(dev, dist, dist_len, fbuf, fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);

                    if (calc_len == 0) {
                        printf("the frame is not valid !\n");
                    }
                    (void)calc_len;


                    /* decode one frame to pcl */
                    {
                        int pcl_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
						float *pcl = new float[pcl_len * 4];	//
                        int calc_len = dmcam_frame_get_pcl_xyzd(dev, pcl, pcl_len*4, dist, dist_len, w, h, true, NULL);

                        /* process distance data */
                        printf("proc pcl data ....\n"); 
                        (void)calc_len;

						delete[] pcl;
                    }
					delete[] dist;
                }
                /* decode one frame to gray */
                {
                    int gray_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
					float *gray = new float[gray_len];
                    int calc_len = dmcam_frame_get_gray(dev, gray, gray_len, fbuf, fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);

                    assert(calc_len == gray_len);
                    printf("proc gray data ....\n"); 

                    /* process gray data */
                    (void)calc_len;

					delete[] gray;
                }

            }
			delete[] fbuf;

            /* stop capturing */
            dmcam_cap_stop(dev);
        }

        /* close device */
        dmcam_dev_close(dev);
    }
FINAL:

    /* uninit dmcam driver */
    dmcam_uninit();
    return 0;
}

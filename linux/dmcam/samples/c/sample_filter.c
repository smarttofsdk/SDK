/*****************************************************************//**
 *       @file  sample_filter.c
 *      @brief  Brief Decsription
 *
 *  Detail Decsription starts here
 *
 *   @internal
 *     Project  $Project$
 *     Created  12/20/2017 
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

#include "dmcam.h"
#ifdef _WIN32
    #include <windows.h>
#define sleep(s) Sleep(s)
#else
#endif
dmcam_dev_t *dev;

/** 
 * @brief:filter function is used to control calibration data
 *       load or not load
 * 
 */
void test_filter(dmcam_dev_t *dev)
{
    dmcam_filter_args_u witem;
    witem.lens_id = 1;
    assert(dmcam_filter_enable(dev,DMCAM_FILTER_ID_LEN_CALIB, &witem,sizeof(dmcam_filter_args_u))); //enable lens calibration.eg.distortion

    witem.case_idx = 2; //factory calibration enable
    assert(dmcam_filter_enable(dev,DMCAM_FILTER_ID_PIXEL_CALIB, &witem,sizeof(dmcam_filter_args_u)));

    witem.case_idx = 3; //user calibration enable
    assert(dmcam_filter_enable(dev,DMCAM_FILTER_ID_PIXEL_CALIB, &witem, sizeof(dmcam_filter_args_u)));

    witem.case_idx = 3; //user calibration disable
    assert(dmcam_filter_disable(dev,DMCAM_FILTER_ID_PIXEL_CALIB));

}
int main(int argc, char **argv)
{
    //int test_times = 10;
    int debug_level = 0;

    if (argc > 1) {
        debug_level = atoi(argv[1]);
        printf(" debug level set to %d\n", debug_level);
    }
    dmcam_init(NULL);

    dmcam_log_cfg(LOG_LEVEL_INFO, LOG_LEVEL_TRACE, LOG_LEVEL_NONE - debug_level);

    /* open the first device  */
    dev = dmcam_dev_open(NULL); /*Note, first open ,will load calibration data ,wait until the red led stop twinkle*/
    if (!dev) {
        printf(" open device failed\n");
        goto FINAL;
    }

    test_filter(dev);
    /* close device */
    dmcam_dev_close(dev);

FINAL:
    dmcam_uninit();

    return 0;
}








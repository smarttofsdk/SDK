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
	dmcam_filter_id_e filter_id = DMCAM_FILTER_ID_PIXEL_CALIB; //像素校准
	dmcam_filter_enable(dev,filter_id,&witem,sizeof(dmcam_filter_args_u));//开启像素校准
	
	filter_id = DMCAM_FILTER_ID_MEDIAN;	//深度滤波
	witem.median_ksize = 3;	//深度滤波通常设置值
	dmcam_filter_enable(dev,filter_id,&witem,sizeof(dmcam_filter_args_u));//开启深度滤波
	
	filter_id = DMCAM_FILTER_ID_AMP;	//最小幅值滤波
	witem.min_amp = 30;	//设置的最小幅值滤波的阈值
	dmcam_filter_enable(dev,filter_id,&witem,sizeof(dmcam_filter_args_u));//开启最小幅值滤波
	
	filter_id = DMCAM_FILTER_ID_AUTO_INTG;	//自动曝光
	witem.sat_ratio = 5;//自动曝光时设置的值
	dmcam_filter_enable(dev,filter_id,&witem,sizeof(dmcam_filter_args_u));//开启自动曝光
	
	dmcam_filter_disable(dev,DMCAM_FILTER_ID_PIXEL_CALIB); //关闭像素校准
	
	dmcam_filter_disable(dev,DMCAM_FILTER_ID_MEDIAN);	//关闭深度滤波
	
	dmcam_filter_disable(dev,DMCAM_FILTER_ID_AMP);	//关闭最小幅值滤波
	
	dmcam_filter_disable(dev,DMCAM_FILTER_ID_AUTO_INTG);	//关闭自动曝光

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

    dmcam_log_cfg(LOG_LEVEL_INFO, LOG_LEVEL_TRACE, dmcam_log_level_e(LOG_LEVEL_NONE - debug_level));

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








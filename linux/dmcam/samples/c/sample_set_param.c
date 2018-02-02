/*****************************************************************//**
 *       @file  sample_set_param.c
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
#define sleep(s) Sleep(s)
#else
#endif
dmcam_dev_t *dev;
/*------------PARAM RANGE discription(For product TM-TI)-------------------*/

/*@PARAM_DEV_MOD
 0:Normal mode
 1:DFU mode
 0xf0:TEST USB mode
*/
/*@MPARAM_MOD_FREQ(MHz),supported value 
10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25, 
26,27,28,29,30,31,32,33,34,36,38,39,40,42,44,45, 
46,48,50,51,54,57,60,66,69,72,75
*/

/*@PARAM_ROI
 row max value 240
 column max value 320
*/
/*@PARAM_ILLUM_POWER
 
 range[0,100]
*/
/*@PARAM_FRAME_RATE
 
 
*/
/*@PARAM_INTG_TIME
 
 range[33,1292]us 
*/

/*------------PARAM RANGE discription(For product TM-EX)-------------------*/
/*@PARAM_DEV_MOD
 0:Normal mode
 1:DFU mode
 0xf0:TEST USB mode
*/
/*@MPARAM_MOD_FREQ(MHz),supported value 
typedef enum {
    MOD_FREQ_24000KHZ,//24MHz
    MOD_FREQ_12000KHZ,//12MHz 
    MOD_FREQ_6000KHZ,//6MHz 
    MOD_FREQ_3000KHZ,//3MHz 
    MOD_FREQ_1500KHZ,//1.5MHz
    MOD_FREQ_750KHZ,//0.75MHz 
    NumberOfTypes
} mod_freq_t; 
*/

/*@PARAM_ROI
 row max value 240
 column max value 320
*/
/*@PARAM_ILLUM_POWER
 not support at this version
*/
/*@PARAM_FRAME_RATE
 45fps@320*240(integration time=700us)
 
*/
/*@PARAM_INTG_TIME
 
 range[1,3000]us 
*/



void test_param_rw(dmcam_dev_t *dev)
{
    dmcam_param_item_t wparam[7];
    dmcam_param_item_t rparam[7];
    /*param set*/
    uint32_t mfreq = 24000000; //Hz
    uint32_t mode = DEV_MODE_NORMAL;

    wparam[0].param_id = PARAM_MOD_FREQ;
    memcpy(&wparam[0].param_val.mod_freq, &mfreq, sizeof(mfreq));
    wparam[0].param_val_len = sizeof(mfreq);

    wparam[1].param_id = PARAM_DEV_MODE;
    memcpy(&wparam[1].param_val.dev_mode, &mode, sizeof(mode));
    wparam[1].param_val_len = sizeof(mode);
    //param set
    //  printf("size:%d,%d\n", (int)sizeof(dmcam_param_item_t), (int)sizeof(PARAM_INFO_SERIAL));
    printf("->test set 2 param\n");
    assert(dmcam_param_batch_set(dev, &wparam[0], 2));
    //  assert(dmcam_param_batch_set(dev, wparam, 2));
    //  printf("->test set 5 param\n");
    //  assert(dmcam_param_batch_set(dev, wparam, 5));
    //param get
    sleep(1000); //wait cfg read
    /*param get*/
    memset(rparam, 0, sizeof(rparam));
    rparam[0].param_id = PARAM_MOD_FREQ;
    rparam[0].param_val_len = sizeof(mfreq);
    rparam[1].param_id = PARAM_DEV_MODE;
    rparam[1].param_val_len = sizeof(mode);

    rparam[2].param_id = PARAM_INFO_VENDOR;
    rparam[3].param_id = PARAM_INFO_PRODUCT;
    rparam[4].param_id = PARAM_INFO_CAPABILITY;
    rparam[5].param_id = PARAM_INFO_SERIAL;
    rparam[6].param_id = PARAM_INFO_VERSION;


    //  assert(dmcam_param_batch_get(dev, &rparam[0], 1)); //Get one param
    printf("Test get 7 param\n");
    assert(dmcam_param_batch_get(dev, &rparam[0], 7)); //Get 6 param
                                                       //  assert(dmcam_param_batch_get(dev, &rparam[4], 3)); //Get 6 param
    memcpy(&mfreq, &rparam[0].param_val.mod_freq, sizeof(mfreq));
    memcpy(&mode, &rparam[1].param_val.dev_mode, sizeof(mode));
    printf("Get freq:%d\n", mfreq);
    printf("Get mode:%d\n", mode);
    // assert(memcmp(&wparam[0], &rparam[0], 9) == 0);
    // assert(memcmp(&wparam[1], &rparam[1], 9) == 0);
    printf("Vendor:%s\n", rparam[2].param_val.info_vendor);
    printf("Product info:%s\n", rparam[3].param_val.info_product);
    printf("PARAM_INFO_CAPABILITY(len=%d): max of w,h,d,fps,intg=%d,%d,%d,%d,%d\n",
           rparam[4].param_val_len,
           rparam[4].param_val.info_capability.max_frame_width,
           rparam[4].param_val.info_capability.max_frame_height,
           rparam[4].param_val.info_capability.max_frame_depth,
           rparam[4].param_val.info_capability.max_fps,
           rparam[4].param_val.info_capability.max_intg_us
          );
    {
        uint32_t id[3];

        assert(sizeof(id) == rparam[5].param_val_len);
        memcpy(id, rparam[5].param_val.info_serial.serial, sizeof(id));
        printf("PARAM_INFO_SERIAL(len=%d): %x,%x,%x\n", rparam[5].param_val_len,
               rparam[5].param_val.info_serial.serial[0],
               rparam[5].param_val.info_serial.serial[1],
               rparam[5].param_val.info_serial.serial[2]
              );
    }
    {
        uint16_t ver[4]; //hw_ver, st_ver, tfc_ver,tfc_hw_ver;

        assert(sizeof(ver) == rparam[6].param_val_len);
        memcpy(ver, &rparam[6].param_val, rparam[6].param_val_len);
        printf("hw_ver:%d,st_ver:%d,tfc_ver:%d\n", ver[0], ver[1], ver[2]);
    }
#if 0
    //test frame fmt set
    uint32_t fmt = FRAME_FMT_DISTANCE;

    wparam[0].param_id = PARAM_FRAME_FORMAT;
    memcpy(&wparam[0].param_val.frame_format, &fmt, sizeof(fmt));
    //  printf("fmt:%d,id:%d\n", fmt, wparam[0].param_id);
    wparam[0].param_val_len = sizeof(fmt);
    assert(dmcam_param_batch_set(dev, &wparam[0], 1));
    rparam[0].param_id = PARAM_FRAME_FORMAT;     //set read id
    assert(dmcam_param_batch_get(dev, &rparam[0], 1));
    printf("fmt:%d,id:%d\n", rparam[0].param_val.frame_format.format, rparam[0].param_id);
    assert(wparam[0].param_val.frame_format.format == rparam[0].param_val.frame_format.format);
#endif
}

void test_param_read(dmcam_dev_t *dev)
{
    dmcam_param_item_t rparam[PARAM_ENUM_COUNT];
    int i;

    memset(rparam, 0, sizeof(rparam));
    /* read all params */
    for (i = 0; i < PARAM_ENUM_COUNT; i++) {
        rparam[i].param_id = (dmcam_dev_param_e)i;
    }
    assert(dmcam_param_batch_get(dev, rparam, PARAM_ENUM_COUNT));

    {
        /* printing the params */

        /* PARAM_DEV_MODE : len=4 */
        assert(rparam[PARAM_DEV_MODE].param_val_len == sizeof(rparam[PARAM_DEV_MODE].param_val.dev_mode));
        printf("PARAM_DEV_MODE(len=%d): %d\n",
               rparam[PARAM_DEV_MODE].param_val_len, rparam[PARAM_DEV_MODE].param_val.dev_mode);

        /* PARAM_MOD_FREQ : len=4 */
        assert(rparam[PARAM_MOD_FREQ].param_val_len == sizeof(rparam[PARAM_MOD_FREQ].param_val.mod_freq));
        printf("PARAM_MOD_FREQ(len=%d): %d\n", rparam[PARAM_MOD_FREQ].param_val_len, rparam[PARAM_MOD_FREQ].param_val.mod_freq);

        /* PARAM_INFO_VENDOR : len=4 */
        assert(rparam[PARAM_INFO_VENDOR].param_val_len > 0);
        printf("PARAM_INFO_VENDOR(len=%d): %s\n", rparam[PARAM_INFO_VENDOR].param_val_len, rparam[PARAM_INFO_VENDOR].param_val.info_vendor);
        /* PARAM_INFO_PRODUCT : len=4 */
        assert(rparam[PARAM_INFO_PRODUCT].param_val_len > 0);
        printf("PARAM_INFO_PRODUCT(len=%d): %s\n", rparam[PARAM_INFO_PRODUCT].param_val_len, rparam[PARAM_INFO_PRODUCT].param_val.info_product);
        /* PARAM_INFO_CAPABILITY : len=4 */
        assert(rparam[PARAM_INFO_CAPABILITY].param_val_len == sizeof(rparam[PARAM_INFO_CAPABILITY].param_val.info_capability));
        printf("PARAM_INFO_CAPABILITY(len=%d): max of w,h,d,fps,intg=%d,%d,%d,%d,%d\n",
               rparam[PARAM_INFO_CAPABILITY].param_val_len,
               rparam[PARAM_INFO_CAPABILITY].param_val.info_capability.max_frame_width,
               rparam[PARAM_INFO_CAPABILITY].param_val.info_capability.max_frame_height,
               rparam[PARAM_INFO_CAPABILITY].param_val.info_capability.max_frame_depth,
               rparam[PARAM_INFO_CAPABILITY].param_val.info_capability.max_fps,
               rparam[PARAM_INFO_CAPABILITY].param_val.info_capability.max_intg_us
              );
        /* PARAM_INFO_SERIAL : */
        assert(rparam[PARAM_INFO_SERIAL].param_val_len == sizeof(rparam[PARAM_INFO_SERIAL].param_val.info_serial));
        printf("PARAM_INFO_SERIAL(len=%d): %x,%x,%x\n", rparam[PARAM_INFO_SERIAL].param_val_len,
               rparam[PARAM_INFO_SERIAL].param_val.info_serial.serial[0],
               rparam[PARAM_INFO_SERIAL].param_val.info_serial.serial[1],
               rparam[PARAM_INFO_SERIAL].param_val.info_serial.serial[2]
              );
        /* PARAM_INFO_DEV : */
        assert(rparam[PARAM_INFO_VERSION].param_val_len == sizeof(rparam[PARAM_INFO_VERSION].param_val.info_version));
        printf("PARAM_INFO_VERSION(len=%d): hwver:%u,swver:%u,sw2ver:%u,hw2ver:%u\n", rparam[PARAM_INFO_VERSION].param_val_len,
               rparam[PARAM_INFO_VERSION].param_val.info_version.hw_ver,
               rparam[PARAM_INFO_VERSION].param_val.info_version.sw_ver,
               rparam[PARAM_INFO_VERSION].param_val.info_version.sw2_ver,
               rparam[PARAM_INFO_VERSION].param_val.info_version.hw2_ver
              );
        /* PARAM_FRAME_FORMAT : */
        assert(rparam[PARAM_FRAME_FORMAT].param_val_len == sizeof(rparam[PARAM_FRAME_FORMAT].param_val.frame_format));
        printf("PARAM_FRAME_FORMAT(len=%d): %u\n", rparam[PARAM_FRAME_FORMAT].param_val_len,
               rparam[PARAM_FRAME_FORMAT].param_val.frame_format.format);
        /* PARAM_INFO_ILLUM_POWER */
        assert(rparam[PARAM_ILLUM_POWER].param_val_len == sizeof(rparam[PARAM_ILLUM_POWER].param_val.illum_power));
        printf("PARAM_ILLUM_POWER(len=%d): %u\n", rparam[PARAM_ILLUM_POWER].param_val_len,
               rparam[PARAM_ILLUM_POWER].param_val.illum_power.percent);
        /* PARAM_INFO_FRAME_RATE */
        assert(rparam[PARAM_FRAME_RATE].param_val_len == sizeof(rparam[PARAM_FRAME_RATE].param_val.frame_rate));
        printf("PARAM_FRAME_RATE(len=%d): %u\n", rparam[PARAM_FRAME_RATE].param_val_len,
               rparam[PARAM_FRAME_RATE].param_val.frame_rate.fps);
        /* PARAM_INFO_INTG_TIME */
        assert(rparam[PARAM_INTG_TIME].param_val_len == sizeof(rparam[PARAM_INTG_TIME].param_val.intg));
        printf("PARAM_INTG_TIME(len=%d): %u\n", rparam[PARAM_INTG_TIME].param_val_len,
               rparam[PARAM_INTG_TIME].param_val.intg.intg_us);
    }

    //
    //    /*param set*/
    //    uint32_t mfreq = 24000000;
    //uint32_t mode = DEV_MODE_NORMAL;
    //
    //wparam[0].param_id = PARAM_MOD_FREQ;
    //memcpy(wparam[0].param_val, &mfreq, sizeof(mfreq));
    //wparam[0].param_val_len = sizeof(mfreq);
    //
    //wparam[1].param_id = PARAM_DEV_MODE;
    //memcpy(wparam[1].param_val, &mode, sizeof(mode));
    //wparam[1].param_val_len = sizeof(mode);
    ////param set
    ////  printf("size:%d,%d\n", (int)sizeof(dmcam_param_item_t), (int)sizeof(PARAM_INFO_SERIAL));
    //printf("->test set 2 param\n");
    //assert(dmcam_param_batch_set(dev, &wparam[0], 2));
    ////  assert(dmcam_param_batch_set(dev, wparam, 2));
    ////  printf("->test set 5 param\n");
    ////  assert(dmcam_param_batch_set(dev, wparam, 5));
    ////param get
    //usleep(1000); //wait cfg read
    ///*param get*/
    //memset(rparam, 0, sizeof(rparam));
    //rparam[0].param_id = PARAM_MOD_FREQ;
    //rparam[0].param_val_len = sizeof(mfreq);
    //rparam[1].param_id = PARAM_DEV_MODE;
    //rparam[1].param_val_len = sizeof(mode);
    //
    //rparam[2].param_id = PARAM_INFO_VENDOR;
    //rparam[3].param_id = PARAM_INFO_PRODUCT;
    //rparam[4].param_id = PARAM_INFO_MODEL;
    //rparam[5].param_id = PARAM_INFO_SERIAL;
    //rparam[6].param_id = PARAM_INFO_DEV;
    //
    //
    ////  assert(dmcam_param_batch_get(dev, &rparam[0], 1)); //Get one param
    //printf("Test get 7 param\n");
    //assert(dmcam_param_batch_get(dev, &rparam[0], 7)); //Get 6 param
    //                                                   //  assert(dmcam_param_batch_get(dev, &rparam[4], 3)); //Get 6 param
    //memcpy(&mfreq, rparam[0].param_val, sizeof(mfreq));
    //memcpy(&mode, rparam[1].param_val, sizeof(mode));
    //printf("Get freq:%d\n", mfreq);
    //printf("Get mode:%d\n", mode);
    //assert(memcmp(&wparam[0], &rparam[0], 9) == 0);
    //assert(memcmp(&wparam[1], &rparam[1], 9) == 0);
    //printf("Vendor:%s\n", rparam[2].param_val);
    //printf("Product info:%s\n", rparam[3].param_val);
    //printf("Model:%s\n", rparam[4].param_val);
    //{
    //    uint32_t id[3];
    //
    //    assert(sizeof(id) == rparam[5].param_val_len);
    //    memcpy(id, rparam[5].param_val, sizeof(id));
    //    printf("Serial:%d %d %d\n", id[0], id[1], id[2]);
    //}
    //{
    //    uint16_t ver[3]; //hw_ver, st_ver, tfc_ver;
    //
    //    assert(sizeof(ver) == rparam[6].param_val_len);
    //    memcpy(ver, rparam[6].param_val, rparam[6].param_val_len);
    //    printf("hw_ver:%d,st_ver:%d,tfc_ver:%d\n", ver[0], ver[1], ver[2]);
    //}
#if 1
    //test frame fmt set
    //uint32_t fmt = FRAME_FMT_DISTANCE;
    //
    //wparam[0].param_id = PARAM_INFO_FRM_FMT;
    //memcpy(&wparam[0].param_val[0], &fmt, sizeof(fmt));
    //printf("fmt:%d,id:%d\n", fmt, wparam[0].param_id);
    //wparam[0].param_val_len = sizeof(fmt);
    //assert(dmcam_param_batch_set(dev, &wparam[0], 1));
    //rparam[0].param_id = PARAM_INFO_FRM_FMT;     //set read id
    //assert(dmcam_param_batch_get(dev, &rparam[0], 1));
    //assert(wparam[0].param_val == wparam[0].param_val);
#endif
}

/** 
 * 
 * 
 * @param dev: 
 *          srow :[0,322],must be even ;erow[5-327] must be odd
 *          scol:[0-124]must be even ;ecol[1-125]must be odd
                                             */
void test_frame_size(dmcam_dev_t *dev)
{
    dmcam_param_item_t wparam[1];
    dmcam_param_item_t rparam[1];
    dmcam_param_roi_t sroi;
    dmcam_param_roi_t groi;

    sroi.srow = 0;
    sroi.erow = 79;
    sroi.scol = 0;
    sroi.ecol = 79;

    wparam[0].param_id = PARAM_ROI;
    memcpy(wparam[0].param_val.raw, &sroi, sizeof(sroi));
    wparam[0].param_val_len = sizeof(sroi);
    printf("set roi:(%d,%d),(%d,%d)\n", sroi.srow, sroi.erow, sroi.scol, sroi.ecol);
    assert(dmcam_param_batch_set(dev, &wparam[0], 1));
    rparam[0].param_id = PARAM_ROI;     //set read id
    assert(dmcam_param_batch_get(dev, &rparam[0], 1));
    memcpy(&groi, rparam[0].param_val.raw, rparam[0].param_val_len);
    //   assert(groi.cur_fsize == (sroi.erow - sroi.srow) * (sroi.ecol - sroi.scol) * 4);
    printf("frame size:%d\n", groi.cur_fsize);
    printf("max frame size:%d\n", groi.max_fsize);
    printf("len:%d get roi:(%d,%d),(%d,%d)\n",  rparam[0].param_val_len, groi.srow, groi.erow, groi.scol, groi.ecol);
#if 0
    dmcam_roi_t groi; //get use
    dmcam_roi_t sroi; //set use
    assert(dmcam_get_frame_size(dev, &groi));
    printf("frame size:%d\n", groi.cur_fsize);
    sroi.scol = 16;
    sroi.ecol = 160;
    sroi.srow = 16;
    sroi.erow = 160;
    printf("set roi:(%d,%d),(%d,%d)\n", sroi.scol, sroi.ecol, sroi.erow, sroi.ecol);
    assert(dmcam_set_frame_size(dev, &sroi));
    assert(dmcam_get_frame_size(dev, &groi));
    assert(groi.cur_fsize == (sroi.erow - sroi.srow)*(sroi.ecol-sroi.scol) * 4);
    printf("frame size:%d, pixel size:%d bytes\n", groi.cur_fsize,groi.pix_width);
    printf("max frame size:%d\n", groi.max_fsize);
    printf("get roi:(%d,%d),(%d,%d)\n", groi.scol, groi.ecol, groi.erow, groi.ecol);
#endif

}


/** 
 * 
 * 
 * @param dev
 * @param intg_time_us:range[33-1292]us
 */
void test_param_info_intg_time(dmcam_dev_t *dev, uint16_t intg_time_us)
{
    dmcam_param_item_t wparam;
    dmcam_param_item_t rparam;
#if 0
    uint8_t duty_cycle = intg_time_percent; //%

    assert(duty_cycle <= 100);
    memset(&wparam, 0, sizeof(wparam));
    memset(&rparam, 0, sizeof(rparam));
    wparam.param_id = PARAM_INTG_TIME;
    wparam.param_val_len = sizeof(duty_cycle);
    memcpy(&wparam.param_val, &duty_cycle, sizeof(duty_cycle));

    assert(dmcam_param_batch_set(dev, &wparam, 1));
    wparam.param_id = PARAM_INTG_TIME;
    assert(dmcam_param_batch_get(dev, &rparam, 1));
    assert(memcmp(&duty_cycle, &rparam.param_val, sizeof(duty_cycle)));
    printf("Set intg time %d%% ok\n", intg_time_percent);
#endif
    //using microseconds instead of percent
    uint16_t intg_time = intg_time_us; //%

    assert(intg_time <= 1292 && intg_time >= 33);
    memset(&wparam, 0, sizeof(wparam));
    memset(&rparam, 0, sizeof(rparam));
    wparam.param_id = PARAM_INTG_TIME;
    wparam.param_val_len = sizeof(intg_time);
    wparam.param_val.intg.intg_us = intg_time;

    assert(dmcam_param_batch_set(dev, &wparam, 1));
    rparam.param_id = PARAM_INTG_TIME;
    assert(dmcam_param_batch_get(dev, &rparam, 1));
    printf("Set intg time(%d) %d us ok\n", intg_time, rparam.param_val.intg.intg_us);
}

/** 
 * 
 * 
 * @param dev
 * @param power:range[0-100]
 */
void test_param_info_illum_power(dmcam_dev_t *dev, uint8_t power)
{
    dmcam_param_item_t wparam;
    dmcam_param_item_t rparam;

    assert(power <= 100);
    //  uint8_t duty_cycle = power; //%

    memset(&wparam, 0, sizeof(wparam));
    memset(&rparam, 0, sizeof(rparam));
    wparam.param_id = PARAM_ILLUM_POWER;
    wparam.param_val_len = 1;
    wparam.param_val.raw[0] = power;

    assert(dmcam_param_batch_set(dev, &wparam, 1));
    rparam.param_id = PARAM_ILLUM_POWER;
    assert(dmcam_param_batch_get(dev, &rparam, 1));
    printf("Set power %hu%% %dok\n", rparam.param_val.illum_power.percent, rparam.param_val_len);
    //    assert(rparam.param_val.illum_power.percent == power);
    printf("Set power %d%% ok\n", power);
}

/** 
 * 
 * @note:just support 30 and 60 fps
 * @param dev
 * @param frame_rate:MAX value is 60
 */
void test_param_info_frame_rate(dmcam_dev_t *dev, uint8_t frame_rate)
{
    dmcam_param_item_t wparam;
    dmcam_param_item_t rparam;

    assert(frame_rate <= 60);
    uint8_t rate = frame_rate; //%

    memset(&wparam, 0, sizeof(wparam));
    memset(&rparam, 0, sizeof(rparam));
    wparam.param_id = PARAM_FRAME_RATE;
    wparam.param_val_len = sizeof(rate);
    memcpy(&wparam.param_val, &rate, sizeof(rate));

    assert(dmcam_param_batch_set(dev, &wparam, 1));
    wparam.param_id = PARAM_FRAME_RATE;
    assert(dmcam_param_batch_get(dev, &rparam, 1));
    assert(memcmp(&rate, &rparam.param_val, sizeof(rate)));
    printf("Set frame rate %d ok\n", frame_rate);
}
struct phase_corr {
    uint16_t corr1;
    uint16_t corr2;
};
void test_param_phase_corr(dmcam_dev_t *dev, struct phase_corr *corr)
{
    dmcam_param_item_t wparam;
    dmcam_param_item_t rparam;

    assert(corr->corr1 <= 4095 && corr->corr2 <= 4095);

    memset(&wparam, 0, sizeof(wparam));
    memset(&rparam, 0, sizeof(rparam));
    wparam.param_id = PARAM_PHASE_CORR;
    wparam.param_val_len = sizeof(struct phase_corr);
    memcpy(&wparam.param_val.phase_corr, corr, sizeof(struct phase_corr));
    printf("Set phase corr(%d,%d)\n", wparam.param_val.phase_corr.corr1, wparam.param_val.phase_corr.corr2);
    assert(dmcam_param_batch_set(dev, &wparam, 1));
    rparam.param_id = PARAM_PHASE_CORR;
    assert(dmcam_param_batch_get(dev, &rparam, 1));
    printf("Get phase corr(%d,%d)ok\n", rparam.param_val.phase_corr.corr1, rparam.param_val.phase_corr.corr2);
    //    assert(rparam.param_val.illum_power.percent == power);

}

void test_get_temperature(dmcam_dev_t *dev)
{
    dmcam_param_item_t rparam;
    int16_t temps[4];

    memset(&rparam, 0, sizeof(rparam));
    rparam.param_id = PARAM_TEMP;
    assert(dmcam_param_batch_get(dev, &rparam, 1));
    memcpy(temps, &rparam.param_val.temp, sizeof(temps));
    printf("tl:%.2f C tr:%.2f C\n", temps[0] / 10.0, temps[1] / 10.0);
    printf("bl:%.2f C br:%.2f C\n", temps[2] / 10.0, temps[3] / 10.0);
}

void test_cfg_mod_freq(dmcam_dev_t *dev)
{
    dmcam_param_item_t wparam;
    dmcam_param_item_t rparam;
    int i;
    uint32_t mod_freq = 24000000;
    uint32_t mod_freq_items[] = {
        24000000, //24MHz
        12000000,
        6000000,
        3000000,
        1500000,
        750000,
    };
    for (i = 0; i < sizeof(mod_freq_items) / sizeof(mod_freq_items[0]); i++) {
        memset(&wparam, 0, sizeof(wparam));
        memset(&rparam, 0, sizeof(rparam));
        wparam.param_id = PARAM_MOD_FREQ;
        wparam.param_val_len = sizeof(mod_freq);
        wparam.param_val.mod_freq = mod_freq_items[i];
        printf("Set mod_freq:%d\n", mod_freq);
        assert(dmcam_param_batch_set(dev, &wparam, 1));
        rparam.param_id = PARAM_MOD_FREQ;
        assert(dmcam_param_batch_get(dev, &rparam, 1));
        printf("Get MOD_FREQ:%d ok\n", rparam.param_val.mod_freq);
        assert(rparam.param_val.mod_freq == mod_freq_items[i]);
    }

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
    dev = dmcam_dev_open(NULL);
    if (!dev) {
        printf(" open device failed\n");
        goto FINAL;
    }
    test_cfg_mod_freq(dev);
    //test_param_read(dev);
    /*  while (1) {
          test_param_info_intg_time(dev, 400); //auto adjust
          usleep(1000 * 100);
      }*/
    /* test_param_info_illum_power(dev, 40);
     test_frame_size(dev);
     test_param_rw(dev);
     {
         struct phase_corr pcorr;
         pcorr.corr1 = 2000;
         pcorr.corr2 = 2000;
         test_param_phase_corr(dev, &pcorr);
     }
     test_param_info_frame_rate(dev, 60);
     */
    sleep(5);
    test_get_temperature(dev);
    /* close device */
    dmcam_dev_close(dev);

FINAL:
    dmcam_uninit();

    return 0;
}

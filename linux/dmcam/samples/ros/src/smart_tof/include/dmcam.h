/*****************************************************************//**
 *       @file  dmcam.h
 *      @brief  DM's camera device API
 *
 *  Detail Decsription starts here
 *
 *   @internal
 *     Project  $Project$
 *     Created  3/13/2017 
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

#ifndef DMCAM_H
#define DMCAM_H
#ifdef __cplusplus
extern "C"
{
#endif
#ifdef LIBDMCAM_DLL_EXPORTS
#define __API __declspec(dllexport)
#else
#define __API //__declspec(dllimport)
#endif
#include <stdbool.h>
#include <stdint.h>

#define DM_NAME "DMCAM"
#define DM_VERSION_MAJOR 2
#define DM_VERSION_MINOR 100
#define DM_VERSION_STR "v2.100"

#define DMCAM_ERR_CAP_FRAME_DISCARD (3)
#define DMCAM_ERR_CAP_WRONG_STATE (-2)
#define DMCAM_ERR_CAP_TIMEOUT (-5)
#define DMCAM_ERR_CAP_STALL   (-7)
#define DMCAM_ERR_CAP_ERROR   (-8)
#define DMCAM_ERR_CAP_UNKNOWN  (-10)

#define DMCAM_FILTER_EN 1
/**
 * dmcam device structure. It describes device usb port info, 
 * device info 
 */
typedef struct {
    void *handler;

    uint8_t usb_bus_num;
    uint8_t usb_port_num;
    uint8_t usb_dev_addr;
    uint8_t usb_speed;

    char product[32];
    char vendor[32];
    char serial[32];

    void *lock; // device lock

    void *user_data0; // used internally for python extension
    void *user_data1; // used internally for python extension
    bool init_flag;  /* struct init flag */
    bool alloc_flag; /* malloc flag used internally */
    bool api_flag; /* sync between frame wait/get and normal api */
}dmcam_dev_t;

/**
 * camera parameters
 */
typedef struct {
    float cx;           //center point x
    float cy;           //center point y
    float fx;           //focal length x
    float fy;           //focal length y
    float scale;    //ratio between image value and real distance
}dmcam_camera_para_t;

typedef enum {
    LOG_LEVEL_TRACE = 0,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_NONE,
}dmcam_log_level_e;

typedef enum
{
    DEV_RST_DEV = 0,
    DEV_RST_CAM0 = 1,
    DEV_RST_CAM1 = 2,
    DEV_RST_USB = 3,
    DEV_RST_IR1 = 4,
    DEV_RST_IR2 = 5,
    DEV_RST_MCU = 8,
    //-----
    DEV_RST_CNT,
} dmcam_dev_rst_e;

typedef enum
{
    DEV_MODE_NORMAL = 0, //3D mode
    DEV_MODE_DFU,
    DEV_MODE_TEST = 8,
} dmcam_dev_mode_e;

typedef enum  {
    DEV_REG_RESERVED = 0,
    DEV_REG_CAM0 = 1,
    DEV_REG_CAM1 = 2,
    DEV_REG_IR_0 = 4,
    DEV_REG_IR_1 = 5,
    DEV_REG_MCU = 8
}dmcam_dev_reg_e;

//typedef enum {
//    FRAME_FMT_DCS1,
//    FRAME_FMT_DCS4,
//}dmcam_frame_format_e;

typedef enum {
    PARAM_DEV_MODE = 0,
    PARAM_MOD_FREQ,

    PARAM_INFO_VENDOR,
    PARAM_INFO_PRODUCT,
    PARAM_INFO_CAPABILITY,
    PARAM_INFO_SERIAL,
    PARAM_INFO_VERSION,  //HW&SW info

    PARAM_ROI, //ROI set/get
    PARAM_FRAME_FORMAT, //frame information,eg.dcs1for gray,4 dcs for distance
    PARAM_ILLUM_POWER, //illumination power set/get
    PARAM_FRAME_RATE, //frame rate set/get
    PARAM_INTG_TIME, //integration time set/get
    PARAM_PHASE_CORR, //phase offset correction
                      //PARAM_SWITCH_MODE, /*>swith mode use[gray,3d]*/
    PARAM_TEMP,        //<Get camera temperature--------------
    PARAM_ENUM_COUNT,
}dmcam_dev_param_e;

typedef enum {
    BIN_DATA_TYPE_MCU, //mcu firmware
    BIN_DATA_TYPE_TFC, //TOF controller IC firmware
    BIN_DATA_TYPE_CALIB, //>TOF calibration data
}dmcam_bin_data_type_e;
/** 
 * Frame size and Max frame size can be get from paramter 
 * interface using the PRAM_INFO_ROI parameter. 
 *  
 * @note: For PARAM_INFO_ROI, it's format like follows. 
 *  (srow,scol)_ _ _ _ _ _ _ _ _
 *            |                 |
 *            |     ROI         |
 *            |_ _ _ _ _ _ _ _ _|
 *                               (erow,ecol)
 *  frame size = (ecol - scol) * (erow - srow) * pixel_width;
 * 

 */
typedef struct {
    uint16_t srow; /*start address for row,multiple of 16*/
    uint16_t erow; /*end address for row,multiple of 16*/
    uint16_t scol; /*start address for columon,multiple of 16*/
    uint16_t ecol; /*end address for columon,multiple of 16*/
    //   uint8_t pix_width;/*pixel data width,number of bytes*/
    uint32_t cur_fsize; /*current frame size*/
    uint32_t max_fsize; /*MAX frame size*/
}dmcam_param_roi_t;

#pragma pack(push)
#pragma pack(1)
typedef union {
    uint8_t raw[18]; // raw data
    uint32_t dev_mode;
    uint32_t mod_freq;
    char info_vendor[18];
    char info_product[18];
    struct {
        uint16_t max_frame_width;
        uint16_t max_frame_height;
        uint16_t max_frame_depth;
        uint16_t max_fps;
        uint16_t max_intg_us;
    } info_capability;
    struct {
        uint32_t serial[3];
    } info_serial;
    struct {
        uint16_t hw_ver;
        uint16_t sw_ver;
        uint16_t sw2_ver;
        uint16_t hw2_ver;
    } info_version;
    struct {
        uint32_t format;
    } frame_format;
    struct {
        uint32_t fps;
    } frame_rate;
    dmcam_param_roi_t roi;
    struct {
        uint8_t percent;
    } illum_power;
    struct {
        uint16_t intg_us;
    } intg;
    struct {
        uint16_t corr1; //range:[0,4095]
        uint16_t corr2; //range:[0,4095]
    }phase_corr;
    struct {
        int16_t tl_cal;
        int16_t tr_cal;
        int16_t bl_cal;
        int16_t br_cal;
    }temp;
}dmcam_param_val_u;
#pragma pack(pop)

typedef struct {
    dmcam_dev_param_e param_id;
    uint8_t param_val_len;
    dmcam_param_val_u param_val;
} dmcam_param_item_t;


typedef enum {
    DM_FRAME_FMT_RAW = 0,
    DM_FRAME_FMT_DISTANCE,
    DM_FRAME_FMT_GRAY,
    DM_FRAME_FMT_PCLOUD,
    DM_FRAME_FMT_RGB,
    /* --- TBD ----*/
}dmcam_frame_fmt_e;

typedef struct {
    uint32_t frame_size;   // size in bytes
    uint16_t frame_format; // frame format code (sensor dependent)
    uint16_t frame_idx;

    uint32_t width;
    uint32_t height;
    uint8_t depth;
    uint8_t pixel_format;
    uint16_t priv_code;
}dmcam_frame_info_t;

typedef struct {
    //uint32_t frame_format;  /** raw frame format  */
    //int frame_fbpos;   /** frame position in frame buffer */
    //int frame_count;   /** frame count from the start of capturing */
    //int frame_size;    /** frame size in bytes */
    dmcam_frame_info_t frame_info;
    void *frame_data;  /** frame data pointer */
}dmcam_frame_t;

typedef struct {
    uint32_t mod_clk; // modulation clock
    uint16_t fps;     // frame per seconds
    uint16_t priv_mode; // other mode
    uint16_t min_amp;
    uint16_t max_amp;
    char *name;  // name string of the use case
}dmcam_use_case_t;

/** camera frame ready function prototype   */
typedef void (*dmcam_cap_frdy_f)(dmcam_dev_t *dev, dmcam_frame_t *frame);
typedef bool (*dmcam_cap_err_f)(dmcam_dev_t *dev, int err, void *err_args);

/**
 * Init the DM camera layer. It should be called before any 
 * dmcam API is invoked.
 * 
 * @param log_fname [in] specified log file name of dmcam layer. 
 *                  if NULL, the default log
 *                  (dmcam_YYYYMMDD.log) is used.
 *                      
 */

__API void dmcam_init(const char *log_fname);

/**
* Uninit the DM camera layer.
*
*/
__API void dmcam_uninit(void);

/**
 * Set the logging configuration for dmcam layer.
 * 
 * @param console_level [in] specified dmcam_log_level_e, the 
 *                      console log whose log level bellow this
 *                      value will be suppressed.
 * @param file_level [in] specified dmcam_log_level_e, the 
 *                      file log whose log level bellow this
 *                      value will be suppressed.
 * @param usb_level [in] specified dmcam_log_level_e, the 
 *                      usb log whose log level bellow this
 *                      value will be suppressed.
 */

__API void dmcam_log_cfg(dmcam_log_level_e console_level, dmcam_log_level_e file_level, dmcam_log_level_e usb_level);

/**
 * list the dmcam device and fill into dmcam_dev_t array.
 * 
 * @param dev_list [out] device list array to be filled.
 * @param dev_list_num [in] capacity of device list 
 * 
 * @return int [out] number of dmcam device found
 */

__API int dmcam_dev_list(dmcam_dev_t *dev_list, int dev_list_num);

/**
* open specified dmcam device. if the device is not specified,
* it'll try to open the first dmcam device
*
* @param dev [in] specified dmcam device which is usally get
*            from dmcam_dev_list. if Null, the first dmcam
*            device will be opened.
*
* @return dmcam_dev_t* NULL = open device failed.
*/
__API dmcam_dev_t* dmcam_dev_open(dmcam_dev_t *dev);

/**
 * open specified dmcam device with specified fd. this is useful 
 * for android usb device. 
 * 
 * @param fd [in] specified fd
 * 
 * @return LIBDMCAM_DLL_API dmcam_dev_t* NULL = open device 
 *         failed.
 */
__API dmcam_dev_t* dmcam_dev_open_by_fd(int fd);
/**
* Close specified dmcam device.
*
* @param dev
*/
__API void dmcam_dev_close(dmcam_dev_t *dev);

/**
* Reset specified target on the dev
*
* @param dev [in] dmcam device handler
* @param target [in] reset taget defined in dmcam_dev_rst_e
*
* @return bool [out] true = reset ok.
*/
__API bool dmcam_dev_reset(dmcam_dev_t *dev, dmcam_dev_rst_e target);

/**
* Batch write registers of specified target on the device.
*
* @param dev [in] dmcam device handler
* @param target [in] specified target defined in
*               dmcam_dev_reg_e
* @param reg_base [in] base address of the registers
* @param reg_vals [in] register values to be written. All
*                 register value is denoted as UINT32
* @param reg_vals_len [in] count of values in reg_vals
*
* @return bool [out] true = write ok.
*/

__API bool dmcam_reg_batch_write(dmcam_dev_t *dev, dmcam_dev_reg_e target, uint32_t reg_base, const uint32_t *reg_vals, uint16_t reg_vals_len);

/**
* Batch read registers of specified target on the device.
*
* @param dev [in] dmcam device handler
* @param target [in] specified target defined in
*               dmcam_dev_reg_e
* @param reg_base [in] base address of the registers
* @param reg_vals [out] register values to be filled. All
*                 register value is denoted as UINT32
* @param reg_vals_len [in] count of values in reg_vals
*
* @return bool [out] true = read ok.
*/

__API bool dmcam_reg_batch_read(dmcam_dev_t *dev, dmcam_dev_reg_e target, uint32_t reg_base, uint32_t *reg_vals, uint16_t reg_vals_len);

/**
 * Batch write generic parameters to specified device.
 * 
 * @param dev [in] dmcam device handler
 * @param param_items [in] dmcam_param_item_t is used to denotes 
 *             generic parameter:
 *               * param_id[in]: defined in dmcam_dev_param_e to
 *                 identify the parameters.
 *               * param_vals[in]: denotes the generic value
 *                 (max = 16bytes)
 *               * param_vals_len[in]: denotes the length of
 *                 value.
 * @param item_cnt [in] count of params in param_items
 * 
 * @return bool [out] true = operation is ok.
 */

__API bool dmcam_param_batch_set(dmcam_dev_t *dev, const dmcam_param_item_t *param_items, int item_cnt);
/**
* Batch read generic parameters from specified device.
*
* @param dev [in] dmcam device handler
* @param param_items [in/out] dmcam_param_item_t is used to
*             denotes generic parameter:
*               * param_id[in]: defined in dmcam_dev_param_e to
*                 identify the parameters.
*               * param_vals[out]: denotes the generic value
*                 (max = 16bytes) filled by this function
*               * param_vals_len[out]: denotes the length of
*                 value filled by this function.
* @param item_cnt [in] count of params in param_items
*
* @return bool [out] true = operation is ok.
*/
__API bool dmcam_param_batch_get(dmcam_dev_t *dev, dmcam_param_item_t *param_items, int item_cnt);

/**
 * set frame buffer during capturing.
 * 
 * @param dev [in] dmcam device handler
 * @param frame_buf [in] framebuffer assigned by user. if null, 
 *                  the frame_buf will be alloced internally
 * @param frame_buf_size [in] frame buffer size, which will be 
 *                       rouned to frame size boundary.
 * 
 * @return bool [out] return true = set ok.
 */

__API bool dmcam_cap_set_frame_buffer(dmcam_dev_t *dev, uint8_t *frame_buf, uint32_t frame_buf_size);
/**
* get frame buffer settings.
*
* @param dev [in] dmcam device handler
* @param frame_buf [out] framebuffer pointer. can be NULL.
* @param frame_buf_size [out] frame buffer size . can be NULL
*
* @return bool [out] return true = set ok.
*/

__API bool dmcam_cap_get_frame_buffer(dmcam_dev_t *dev, uint8_t **frame_buf, uint32_t *frame_buf_size);

/**
* register frame ready callback function
*
* @param dev [in] dmcam device handler
* @param cb  [in] callback function in following format:
*     void (*dmcam_cap_frdy_f)(dmcam_dev_t*, dmcam_frame_t)
*
*/
__API void dmcam_cap_set_callback_on_frame_ready(dmcam_dev_t *dev, dmcam_cap_frdy_f cb);

/**
* register error callback function. It's invoked when some
* error occurs during the capturing process.
*
* @param dev [in] dmcam device handler
* @param cb [in] callback function in following format:
*     void (*dmcam_cap_err_f)(dmcam_dev_t*, int errno);
*/
__API void dmcam_cap_set_callback_on_error(dmcam_dev_t *dev, dmcam_cap_err_f cb);

/**
* Take a snapshot and fill frame data into specified frame. If
* the device is capturing, the snapshot will return the latest
* image{} or it'll auto start/snapshot/stop
*
* @param dev [in] dmcam device handler
* @param frame_data [out] frame data
* @param frame_dlen [in] frame buffersize should be large
*                   enough to containing one frame.
* @param frame [out] frame_t filled during snapshot. it can be
*              null
* @return bool return true = ok
*/
__API bool dmcam_cap_snapshot(dmcam_dev_t *dev, uint8_t *frame_data, uint32_t frame_dlen, dmcam_frame_t *frame);

/**
 * Check whether the device is in capturing state.
 * 
 * @param dev [in] dmcam device handler
 * 
 * @return bool [out] true = device in capturing state
 */
__API bool dmcam_cap_is_ongoing(dmcam_dev_t *dev);

/**
* start device capturing.
*
* @param dev [in] dmcam device handler
*
* @return bool return true = ok
*/

__API bool dmcam_cap_start(dmcam_dev_t *dev);

/**
* stop device capturing.
*
* @param dev [in] dmcam device handler
*
* @return bool return true = ok
*/

__API bool dmcam_cap_stop(dmcam_dev_t *dev);

/**
* Get specified number of frames into specified user buffer.
* This function may be blocking wait on the frame stream. if
* enough frames data are collected or any error happends, it'll
* returns.
* @param dev [in] dmcam device handler
* @param frame_num [in] number of frames to be captured.
* @param frame_data [out] frame data filled curing capturing.
* @param frame_dlen [in] frame_data buffer size in bytes.
* @param first_frame_info [out] first frame attributes. It can
*                         be NULL
*
* @return int [out] return the number for ready frames
*         collected. On error the errono is returned.
*        (errno < 0)
*/

__API int dmcam_cap_get_frames(dmcam_dev_t *dev, uint32_t frame_num, uint8_t *frame_data, uint32_t frame_dlen, dmcam_frame_t *first_frame_info);

/**
* get one frame into specified buffer. this function is
* non-blocking, if no frame is ready, it returns false
*
* @param dev [in] dmcam device handler
* @param frame_data [out] frame data  to be filled, it can be
*                   NULL
* @param frame_info [out] frame attributes. It can be NULL
*
* @return bool
*/

__API int dmcam_cap_get_frame(dmcam_dev_t *dev, uint8_t *frame_data, uint32_t frame_dlen, dmcam_frame_t *frame_info);
/**
* (Deprecated) wait capture process with specified timeout
*
* @param timeout_msec [in] timeout in miliseconds
*
* @return int 1 = ok, 0 = fail
*/
__API bool dmcam_cap_wait(dmcam_dev_t *dev, int timeout_msec);


/**
* Firmware upgrade for different type target.
*
* @param dev[in]:dmcam device handler
* @param type[in]:firmware type
* @param version[in]:firmware version
* @param file_name[in]:firmware name
*
* @return int
*/
__API int dmcam_firmware_upgrade(dmcam_dev_t *dev, uint8_t type, uint16_t version, const char *file_name);

__API int dmcam_data_download(dmcam_dev_t *dev, char *name, uint8_t type, uint16_t version, uint32_t addr);
__API int dmcam_data_upload(dmcam_dev_t *dev, uint8_t type, const char *file_name);
/**
* covert specified error code into error string
*
* @param error_code
*
* @return const char*
*/

__API const char* dmcam_error_name(int error_code);

/**
 * convert to distance data from raw frame data. 
 * 
 * @param dev [in] specified dmcam device
 * @param dst [out] distance buffer. The unit of distance is in 
 *            meters (float32)
 * @param dst_len [in] distance buffer length in number of float
 * @param src [in] raw frame  data buffer
 * @param src_len [in] raw frame data length in byte
 * @param finfo [in] raw frame information 
 * 
 * @return int [out] return the number for distance points in 
 *         dst
 */

__API int dmcam_frame_get_distance(dmcam_dev_t *dev, float *dst, int dst_len,
                                   uint8_t *src, int src_len, const dmcam_frame_info_t *finfo);

/**
 * get gray data from raw frame data.
 * 
 * @param dev [in] specified dmcam device
 * @param dst [out] distance buffer. The unit of distance is in 
 *            meters (float32)
 * @param dst_len [in] distance buffer length in number of float
 * @param src [in] raw frame  data buffer
 * @param src_len [in] raw frame data length in byte
 * @param finfo [in] raw frame information 
 * 
 * @return int [out] return the number for distance points in 
 *         dst
 */

__API int dmcam_frame_get_gray(dmcam_dev_t *dev, float *dst, int dst_len,
                               uint8_t *src, int src_len, const dmcam_frame_info_t *finfo);

/**
*	depth data to point cloud data
*
*	@param pcl [out] point cloud data after change from depth
*	@param pcl_size	[in] length of pointcloud
*	@param srcdata [in] depth data which should be changed to pointcloud
*   @param img_h [in] rows of the depth
*   @param img_w [in] cols of the depth
*	@param sdatatype [in] the type of srcdata 1:uint8_t 2:uint16_t 3:float
*	@param camera [in] camera intrinsic parameters
*/
__API int dmcam_depth_to_pcl(float *pcl, int pcl_size, uint8_t *srcdata, int img_h, int img_w, uint8_t sdatatype, const dmcam_camera_para_t *camera);
#if DMCAM_FILTER_EN
typedef enum {
    DMCAM_FILTER_ID_LEN_CALIB,  /**>lens calibration*/
    DMCAM_FILTER_ID_PIXEL_CALIB, /**>pixel calibration*/
    DMCAM_FILETER_ID_KALMAN,    /**>Kalman filter for distance data*/
    DMCAM_FILETER_ID_GAUSS,     /**>Gauss filter for distance data*/
    DMCAM_FILTER_ID_AMP, /**>Amplitude filter control*/
    DMCAM_FILTER_CNT,
}dmcam_filter_id_e;

typedef union {
    uint8_t raw[20];
    uint8_t case_idx; /**>User Scenario index */
    uint32_t lens_id; /**>length index*/
    uint32_t min_amp; /**>Min amplitude threshold*/
}dmcam_filter_args_u;


/** 
 *  Enable filter controller setting for raw data processing
 * 
 * @param dev [in] dmcam device handler
 * @param fid [in]:defined in dmcam_filter_id_e to identify the 
 *              filter kind 
 * @param args[in]:denotes filter control args
 * @param args[in]:args value len
 * @return __API int
 */
__API int dmcam_filter_enable(dmcam_dev_t *dev,  dmcam_filter_id_e fid, dmcam_filter_args_u *args, uint32_t arg_len); //dmcam_filter_item_t *items, int item_cnt);
/** 
 *  Disable filter controller setting for raw data processing
 * 
 * @param dev [in] dmcam device handler
 * @param dev [in] dmcam device handler
 * @param fid [in]:defined in dmcam_filter_id_e to identify the 
 *              filter kind 
 * @param args[in]:denotes filter control args
 * 
 * @return __API int
 */
__API int dmcam_filter_disable(dmcam_dev_t *dev,  dmcam_filter_id_e fid);
#endif



typedef enum {
    DMCAM_CMAP_OUTFMT_RGB,
    DMCAM_CMAP_OUTFMT_RGBA,
    DMCAM_CMAP_OUTFMT_BGR,
}dmcam_cmap_outfmt_e;

/**
 * convert mapping float points to pesudo-RGB points with 
 * specified pixel format 
 * 
 * @param dst [out] pseudo-RGB point buffer
 * @param dst_len [in] point buffer size in bytes
 * @param src [in] float points buffer
 * @param src_len [in] count of float points
 * @param outfmt [in] pixel format of the pseudo-RGB 
 * @param min_val [in] minimum range of source point
 * @param max_val [in] max range of source point
 * 
 * @return int [out] the count of pseudo RGB points
 */

__API int dmcam_cmap_float(uint8_t *dst, int dst_len, const float *src, int src_len, dmcam_cmap_outfmt_e outfmt, float min_val, float max_val);

#ifdef __cplusplus
}
#endif
#endif //DMCAM_H

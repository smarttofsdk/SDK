/*******************************************************************
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
#ifdef _MSC_VER
#define __API __declspec(dllexport)

#if _MSC_VER >= 1800
#include <stdbool.h>
#else
#define false 0
#define true 1

#define bool uint8_t
#endif

#else
#define __API
#include <stdbool.h>
#endif
#include <stdint.h>

#define DM_NAME "DMCAM"
#define DM_VERSION_MAJOR 1
#define DM_VERSION_MINOR 81
#define DM_VERSION_REV 4
#define DM_VERSION_STR "v1.81.4"

#define DMCAM_ERR_CAP_FRAME_DISCARD (3)
#define DMCAM_ERR_CAP_WRONG_STATE (-2)
#define DMCAM_ERR_CAP_CANCEL (-3)
#define DMCAM_ERR_CAP_TIMEOUT (-5)
#define DMCAM_ERR_CAP_STALL (-7)
#define DMCAM_ERR_CAP_ERROR (-8)
#define DMCAM_ERR_CAP_EOF (-9)  // cap replay related error
#define DMCAM_ERR_CAP_UNKNOWN (-10)

#define DM_SEEK_SET 0
#define DM_SEEK_CUR 1
#define DM_SEEK_END 2

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5)
#define API_DEPRECATED_FOR(f) __attribute__((deprecated("Use " #f " instead")))
#define API_DEPRECATED \
        __attribute__((deprecated("This API is deprecated and will be remove in the furture release")))
#else
#define API_DEPRECATED_FOR(f)
#define API_DEPRECATED
#endif /* __GNUC__ */

/** camera interface enum   */
typedef enum {
    DEV_IF_USB = 0,
    DEV_IF_ETH,
    DEV_IF_FILE,

    DEV_IF_NONE = 99,
} dmcam_dev_if_e;

struct dmcam_dev_if_info_usb {
    uint8_t usb_bus_num;
    uint8_t usb_port_num;
    uint8_t usb_dev_addr;
    uint8_t usb_speed;
    char product[16];
    char vendor[16];
};
struct dmcam_dev_if_info_eth {
    uint8_t addr[16];
    uint8_t reserved[16];
    uint32_t token;
    uint32_t cid;
};

struct dmcam_dev_if_info_fil {
    void *fd;
    char *fname;
};

typedef struct {
    dmcam_dev_if_e type;  // interface type
    union {
        struct dmcam_dev_if_info_usb usb;
        struct dmcam_dev_if_info_eth eth;
        struct dmcam_dev_if_info_fil fil;
    } info;
} dmcam_dev_if_info_t;

typedef union
{
    uint16_t version[4];
    struct {
        uint16_t hw_ver;
        uint16_t sw_ver;
        uint16_t sw2_ver;
        uint16_t hw2_ver;
    } ver;
} dmcam_dev_ver_u;

/**
 * dmcam device structure. It describes device usb port info,
 * device info
 */
typedef struct {
    void *handler;

    dmcam_dev_if_info_t if_info;

    /*  device info  */
    char product[32];
    char vendor[32];
    char serial[32];

    uint64_t dev_id; /** device uniq id */

    dmcam_dev_ver_u version;

    /*  internal used vars  */
    char *expath; /** extract path to store calibration data,can be set by dmcam_path_cfg */
    void *lock;   /**  device lock*/

    void *user_data0;   /** used internally for python extension */
    void *user_data1;   /** used internally for python extension */
    uint8_t init_flag;  /** struct init flag */
    uint8_t alloc_flag; /** malloc flag used internally */
    uint8_t api_flag;   /** sync between frame wait/get and normal api */
} dmcam_dev_t;

/**
 * camera parameters
 */
typedef struct {
    float cx; /** center point x */
    float cy; /** center point y */
    float fx; /** focal length x */
    float fy; /** focal length y */

    float dcoef[16]; /** distortion coef*/
    uint8_t n_dcoef; /** number of valid distortion coef */
} dmcam_len_param_t;

/** log levels */
typedef enum {
    LOG_LEVEL_TRACE = 0,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_NOTICE1,
    LOG_LEVEL_NOTICE2,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_NONE,
} dmcam_log_level_e;

typedef enum {
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

typedef enum {
    DEV_MODE_NORMAL = 0,  // 3D mode
    DEV_MODE_DFU,
    DEV_MODE_TEST = 8,
    DEV_MODE_DATA_UP,
} dmcam_dev_mode_e;

typedef enum {
    DEV_REG_RESERVED = 0,
    DEV_REG_CAM0 = 1,
    DEV_REG_CAM1 = 2,
    DEV_REG_IR_0 = 4,
    DEV_REG_IR_1 = 5,
} dmcam_dev_reg_e;

typedef enum {
    FRAME_FMT_DCSx1 = 0,                           /** gray                                                 */
    FRAME_FMT_DUAL_MGX_SINE_PIDELAY_DCSx2 = 1,     /** dist DCS*2, sine mode,with pi delay                  */
    FRAME_FMT_SINGLE_MGX_SINE_PIDELAY_DCSx4 = 2,   /** single MGX dist DCS*4, sine mode,with pi dealy       */
    FRAME_FMT_DUAL_MGX_PN_NOPIDELAY_DCSx1 = 3,     /** dist DCS*1, PN mode,without pi delay                 */
    FRAME_FMT_DUAL_MGX_SINE_NOPIDELAY_DCSx1 = 4,   /** dist DCS*1,no sine mode,without pi delay             */
    FRAME_FMT_SINGLE_MGX_PN_PIDELAY_DCSx4 = 5,     /** single MGX, dist DCS*4, PN mode,with pi delay        */
    FRAME_FMT_DUAL_MGX_PN_PIDELAY_DCSx2 = 6,       /** dist DCS*2, PN mode,with pi delay                    */
    FRAME_FMT_SINGLE_MGX_PN_NOPIDELAY_DCSx2 = 7,   /** single MGX dist DCS*2, PN mode,with out pi dealy     */
    FRAME_FMT_SINGLE_MGX_SINE_NOPIDELAY_DCSx2 = 8, /** single MGX dist DCS*2, nosine mode,with out dealy    */
    FRAME_FMT_PHASE = 0x0A,                        /** phase frame data                                     */
    FRAME_FMT_GRAY,                                /** reserved      */
    FRAME_FMT_LOSSY,                               /** reserved     */
    FRAME_FMT_QI = 0x0D,                           /** reserved     */
    FRAME_FMT_QI_DUAL_FREQ = 0x0E,                 /** reserved     */
    FRAME_FMT_HDR_8DCS = 0X0F,                     /** HDR mode     */
    FRAME_FMT_QI_SHIFT_LOSSLESS = 0X10,            /** QI packed mode     */
    FRAME_FMT_QI_SHIFT_LOSSLESS_DUAL_FREQ = 0X11,  /** Dual frequency QI packed mode     */
    FRAME_FMT_CNT,
} dmcam_frame_mode_e;

/** dmcam param ID */
typedef enum {
    PARAM_DEV_MODE = 0,
    PARAM_MOD_FREQ, /** modulation frequency*/

    PARAM_INFO_VENDOR,     /** production vendor information*/
    PARAM_INFO_PRODUCT,    /** production information*/
    PARAM_INFO_CAPABILITY, /** production capability*/
    PARAM_INFO_SERIAL,     /** porduction serials id*/
    PARAM_INFO_VERSION,    /** HW&SW info */
    PARAM_INFO_SENSOR,     /** part version, chip id, wafer id */
    PARAM_INFO_CALIB,      /** get calibration info */

    PARAM_ROI,                 /** ROI set/get*/
    PARAM_FRAME_FORMAT,        /** frame information,eg.dcs1for gray,4 dcs for distance*/
    PARAM_ILLUM_POWER,         /** illumination power set/get*/
    PARAM_FRAME_RATE,          /** frame rate set/get*/
    PARAM_INTG_TIME,           /** integration time set/get*/
    PARAM_PHASE_CORR,          /** phase offset correction*/
    PARAM_TEMP,                /** Get camera temperature--------------*/
    PARAM_HDR_INTG_TIME,       /** Setting HDR integration time param*/
    PARAM_SYNC_DELAY,          /** delay ms for sync use*/
    PARAM_SYS_CALIB_COEFF,     /** system calibration coefficent*/
    PARAM_SYNC_SYS_TIME,       /** set/get module time*/
    PARAM_AMBIENT_LIGHT_COEFF, /** set ambient light calibration coeff. */
    PARAM_DUAL_MOD_FREQ,       /** set mod_freq */

    PARAM_INFO_LENS,       /** Get lens param */
    PARAM_FLIP,            /** image flip*/
    PARAM_RESERVED,        /** rererved*/
    PARAM_INFO_CALIB_FREQ, /** calibration information, discarded*/
    PARAM_DEL_CALIB_DATA,  /** Delete calibration data*/
    PARAM_DEV_CAPTURE_CFG,     /** Capture configuration*/

    PARAM_ENUM_COUNT,
} dmcam_dev_param_e;

typedef enum {
    BIN_DATA_TYPE_MCU,    // mcu firmware
    BIN_DATA_TYPE_TFC,    // TOF controller IC firmware
    BIN_DATA_TYPE_CALIB,  //>TOF calibration data
} dmcam_bin_data_type_e;

/** binning mode */
typedef enum {
    DM_BINNING_1X1,
    DM_BINNING_2X2,
    DM_BINNING_4X4,
    DM_BINNING_8X8,
    DM_BINNING_2X4,
    //---
    DM_BINNING_CNT,
} dmcam_binning_mode_e;

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

#pragma pack(push)
#pragma pack(1)
/** frame roi value   */
typedef struct {
    uint16_t srow;      /** start address for row,multiple of 16*/
    uint16_t erow;      /** end address for row,multiple of 16*/
    uint16_t scol;      /** start address for columon,multiple of 16*/
    uint16_t ecol;      /** end address for columon,multiple of 16*/
    uint8_t binning;    /** binning mode for some sensor*/
    uint32_t max_fsize; /** MAX frame size*/
} dmcam_param_roi_t;

typedef struct dmcam_cap {
    uint16_t max_frame_width;
    uint16_t max_frame_height;
    uint16_t max_frame_depth;
    uint16_t max_fps;
    uint16_t max_intg_us;
    uint8_t illum_board_type;
    uint8_t sensor_board_type;
} dmcam_param_cap_t;

/** dmcam param value */
typedef union
{
    uint8_t raw[18];  // raw data
    uint32_t dev_mode;
    uint32_t mod_freq;

    /* ----  readonly info fields begin ----*/
    char info_vendor[18];  /** camera vendor information*/
    char info_product[18]; /** camera production information*/
    dmcam_param_cap_t info_capability; /** device capablity*/
    struct {
        uint32_t serial[3]; /** serials numbers*/
    } info_serial;
    struct {
        uint16_t hw_ver;  /** hardware1 version*/
        uint16_t sw_ver;  /** firmware1 version*/
        uint16_t sw2_ver; /** firmware2 version*/
        uint16_t hw2_ver; /** hardware2 version*/
    } info_version;

    struct {
        uint16_t part_ver; /** chip part version   */
        uint16_t chip_id;  /** chip id             */
        uint16_t wafer_id; /** wafer id            */
    } chip_info;

    struct {
        uint8_t valid;      /** data is valid;1==valid                         */
        uint8_t flag;       /** 0:no compression used;1:zip compression used   */
        uint32_t ID;        /** data info, version                             */
        uint32_t timestamp; /** calibration date                               */
        uint32_t fsize;     /** head+data+paddingsize                          */
        uint32_t datasize;  /** data size                                      */
    } cinfo;                /** calibration info                               */

    struct {
        float cx; /** center point x */
        float cy; /** center point y */
        float fx; /** focal length x */
        float fy; /** focal length y */
    } lens_param;
    struct {
        uint8_t total_cnt; /** calibration frequency count*/
        uint8_t remain_cnt;
        uint8_t freq[4];
    } calib_freq;
    /* ----  readonly info fields end ----*/
    struct {
        uint32_t format; /** frame format*/
    } frame_format;
    struct {
        uint32_t fps; /** frame rate*/
    } frame_rate;
    struct {
        uint8_t random_delay_en;
        uint16_t delay;
    } sync_delay;
    dmcam_param_roi_t roi; /** image roi*/
    struct {
        uint8_t percent; /** illumination power*/
    } illum_power;
    struct {
        uint16_t intg_us; /** first integration time(normal mode or HDR mode)*/
        uint16_t intg_us_param; /** second integration time in HDR mdoe*/
    } intg;
    struct {
        uint16_t intg_3dhdr; /** second integration time in HDR mdoe*/
    } intg_hdr;
    struct {
        uint16_t corr1;  // range:[0,4095]
        uint16_t corr2;  // range:[0,4095]
    } phase_corr;
    struct {
        int16_t tl_cal;
        int16_t tr_cal;
        int16_t bl_cal;
        int16_t br_cal;
        int16_t ib_cal;
    } temp;
    struct {
        uint32_t mod_freq;    /** system calibration frequency */
        float offset;         /** system calib offset          */
        float coeff;          /** system calib coeff           */
        uint32_t random_freq; /** random frequency*/
        uint8_t valid;        /**  1:coeff is valid; 0: coeff in not valid*/
        uint8_t frm_format;   /** calibration frame format */
    } sys_calib_coeff;
    struct {
        uint32_t sec;
        uint32_t us;
    } sync_time;
    struct {
        uint32_t mod_freq0; /** modulation frequency0*/
        uint32_t mod_freq1; /**  modulation frequency1*/
    } dual_freq;
    struct {
        uint32_t mode;      /** 0:continues , 1: external trig mode*/
        uint32_t frames_per_trig;   /** frames_per_trig frames per trigger, no sense if mode==0*/
    }cap_cfg;
    uint32_t calib_data_type;
    uint16_t ambient_light_coeff;
} dmcam_param_val_u;

#pragma pack(pop)

/**param data struct */
typedef struct {
    dmcam_dev_param_e param_id;  /** param id*/
    uint8_t param_val_len;       /** param length*/
    dmcam_param_val_u param_val; /** param value*/
} dmcam_param_item_t;

/** Framae data format */
typedef enum {
    DM_FRAME_FMT_RAW_DIST = 0, /** distance data without calibration*/
    DM_FRAME_FMT_DISTANCE,     /** distance with calibration*/
    DM_FRAME_FMT_GRAY,         /** Confidence data*/
    DM_FRAME_FMT_PCLOUD,       /** pointcloud data*/
    DM_FRAME_FMT_RGB,          /** reserved*/
    /* --- TBD ----*/
} dmcam_frame_fmt_e;

/**Used for store frame information */
typedef struct {
    uint32_t frame_size;   /**  frame size in bytes*/
    uint16_t frame_format; /**  frame format code (sensor dependent)*/
    uint16_t frame_idx;    /** frame index*/

    uint32_t width;       /** image width*/
    uint32_t height;      /** image height*/
    uint8_t depth;        /** reserved*/
    uint8_t pixel_format; /** internal use*/
    uint16_t priv_code;   /** internal use*/

    int16_t temp0, temp1; /**sensor and illumination board temperature*/
    uint32_t rx_ts;       /** frame rx unix timestamp*/
    uint32_t rx_us;       /** frame rx microseconds*/

    uint8_t dist_is_z;   /** distance is already conv to z*/
    uint8_t reserved[3];
    uint32_t reserved2[16];
} dmcam_frame_info_t;

/**frame data struct */
typedef struct {
    dmcam_frame_info_t frame_info; /** frame informatin*/
    void *frame_data;              /** frame data pointer */
} dmcam_frame_t;

/** camera frame ready function prototype   */
typedef void (*dmcam_cap_frdy_f)(dmcam_dev_t *dev, dmcam_frame_t *frame);
/** camera frame error function prototype   */
typedef bool (*dmcam_cap_err_f)(dmcam_dev_t *dev, int err, void *err_args);

/** capture configuration params */
typedef struct {
    uint32_t cache_frames_cnt; /** max frame count in frame buffer (cache) */
    dmcam_cap_frdy_f
        on_frame_ready;       /** callback when frame is ready in the frame buffer; invoked in dmcam_cap_get_frames */
    dmcam_cap_err_f on_error; /** callback when error happens during capturing; invoked in dmcam_cap_get_frames */
    uint8_t en_save_manually; /** if set to 1, saving is manually controlled by dmcam_cap_save_frame/dist/gray */
    uint8_t en_save_replay;   /** enable saving replay file during capturing. saving happens in dmcam_cap_get_frames */
    uint8_t
        en_save_dist_u16; /** enable saving dist_u16 file during capturing. saving happends in dmcam_cap_get_dist_xx */
    uint8_t
        en_save_gray_u16; /** enable saving gray_u16 file during capturing. saving happends in dmcam_cap_get_gray_xx */
    char *fname_replay;   /** replay file name */
    uint8_t en_fdev_rewind; /** only used when type of the device is reaply-file. if true, capture will auto rewind
                               when the EOF of file is met */
} dmcam_cap_cfg_t;

/** device readonly information. It can be retrieved by
 *  dmcam_dev_get_info() */
typedef struct {
    /** device description string   */
    struct {
        const char *vendor;  /** camera vendor str*/
        const char *product; /** camera production str*/
        const char *serial;  /** camera serial string */

        uint8_t reserved[128]; /** reserved for future use */
    } desc;

    /** device id information  */
    struct {
        uint64_t dev_id;        /** device uniq id */
        const char *uri;        /** device uri */
        uint32_t serial[3];           /** serials numbers*/
        uint8_t reserved[128]; /** reserved for future use */
    }id;

    /** device HW/SW version information  */
    struct {
        uint16_t hw_ver;  /** hardware1 version*/
        uint16_t sw_ver;  /** firmware1 version*/
        uint16_t sw2_ver; /** firmware2 version*/
        uint16_t hw2_ver; /** hardware2 version*/
        uint8_t reserved[64]; /** reserved for future use */
    } ver;

    /** device capability  */
    struct {
        uint16_t max_frame_width;
        uint16_t max_frame_height;
        uint16_t max_frame_depth;
        uint16_t max_fps;
        uint16_t max_intg_us;
        uint8_t reserved[128]; /** reserved for future use */
    } cap;

    /** chip information  */
    struct {
        const char name[16];  /** sensor chip name */
        uint32_t type_id;
        uint32_t uniq_id;
        uint8_t reserved[64]; /** reserved for future use */
    } chip;

    /** calib data information  */
    struct {
        const char *data_fpath; /** calibration data file path. NULL = invalid */
        uint32_t *freq_list;    /** calibrated frequency list */
        uint32_t n_freq;        /** number of calibrated frequencys in list */

        uint8_t flag;           /** 0:no compression used;1:zip compression used   */
        uint32_t timestamp;     /** calibration date                               */
        uint32_t fsize;         /** head+data+paddingsize                          */
        uint32_t datasize;      /** data size                                      */

        uint8_t reserved[256]; /** reserved for future use */
    } calib;

    /** lens information  */
    dmcam_len_param_t lens;
} dmcam_dev_info_t;

/*---------------------------------------------------------------------------*
 * lib init/uninit/cfg API section
 *---------------------------------------------------------------------------*/
/**
 * Init the DM camera layer. It should be called before any
 * dmcam API is invoked.
 *
 * @param log_fname [in] specified log file name of dmcam layer.
 *                  if NULL, the default log
 *                  (dmcam_YYYYMMDD.log) is used.
 *                  if empty string "" is used, no log will
 *                  generated
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
 * set top path for all data files
 * (calibration data, log file ...)
 *
 * @param path [in] specified file system path
 *
 * @return __API void
 */
__API void dmcam_path_set(const char *path);

/**
 * Get top path for all data files
 *
 * @return _API char*
 */
__API const char* dmcam_path_get(void);

/**
 * covert specified error code into error string
 *
 * @param error_code
 *
 * @return const char*
 */
__API const char* dmcam_error_name(int error_code);

/*****************************************************
 *      device enumberation/open/close API section
 *****************************************************/
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
 * @return dmcam_dev_t*  return opened device. NULL = open
 *         device failed.
 */
__API dmcam_dev_t* dmcam_dev_open_by_fd(int fd);

/**
 * open specified dmcam device with specified uri.
 *
 * @param uri_str [in] specified URI. Following URI are
 *  supported:
 *    USB device URI:
 *      usb://bus:port or usb://bus:port:dev_addr
 *    Ethernet device URI:
 *      eth://hwid:token@ip or eth://hwid:token@ip
 *    FILE device URI:
 *      file://filename or filename
 *
 * @return dmcam_dev_t* NULL = open device
 *         failed.
 */
__API dmcam_dev_t* dmcam_dev_open_by_uri(const char *uri_str);

/**
 * Close specified dmcam device.
 *
 * @param dev
 */
__API void dmcam_dev_close(dmcam_dev_t *dev);

/**
 * get URI of specified device.
 *
 * @param dev [in] specified device after dmcam_dev_open
 * @param uri_str [in] uri string buffer
 * @param uri_str_len [in] uri string buffer len
 *
 * @return const char* [out] uri string. If null, get uri
 *         failed.
 */

__API const char* dmcam_dev_get_uri(dmcam_dev_t *dev, char *uri_str, int uri_str_len);


/** 
 * @note This API is available after 1.82.0 
 *  
 * get device readonly information including vendor/product 
 * string, serial, version, calibration, lens .. 
 * 
 * @param dev [in] specified device after dmcam_dev_open
 * 
 * @param info [out] specified info strucure to be filled.
 * 
 * @return specified info structure. NULL = get info failed.
 * 
 */
__API const dmcam_dev_info_t* dmcam_dev_get_info(dmcam_dev_t *dev, dmcam_dev_info_t *info);

/**
 * Reset specified target on the dev
 *
 * @param dev [in] dmcam device handler
 * @param target [in] reset taget defined in dmcam_dev_rst_e
 *
 * @return bool [out] true = reset ok.
 */
__API bool dmcam_dev_reset(dmcam_dev_t *dev, dmcam_dev_rst_e target);

/*****************************************************
 *      register/parameters RW API section
 *****************************************************/
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

__API bool dmcam_reg_batch_write(dmcam_dev_t *dev,
                                 dmcam_dev_reg_e target,
                                 uint32_t reg_base,
                                 const uint32_t *reg_vals,
                                 uint16_t reg_vals_len);

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

__API bool dmcam_reg_batch_read(dmcam_dev_t *dev,
                                dmcam_dev_reg_e target,
                                uint32_t reg_base,
                                uint32_t *reg_vals,
                                uint16_t reg_vals_len);

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

/*---------------------------------------------------------------------------*
 * capture API section
 *---------------------------------------------------------------------------*/

/**
 * Set specified capture configuration for specified device.
 * This api is available from v1.58 to replace @ref
 * dmcam_cap_set_frame_buffer
 *
 * @param dev [in] specified dmcam device
 * @param cfg [in] specified capture configuration
 *
 * @return bool [out] true = set OK.
 */

__API bool dmcam_cap_config_set(dmcam_dev_t *dev, const dmcam_cap_cfg_t *cfg);

/**
 * Get capture configuration of specified device
 *
 * @param dev [in] specified dmcam device
 * @param cfg [out] capture configuration to be filled
 */
__API void dmcam_cap_config_get(dmcam_dev_t *dev, dmcam_cap_cfg_t *cfg);

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

__API int dmcam_cap_get_frames(dmcam_dev_t *dev,
                               uint32_t frame_num,
                               uint8_t *frame_data,
                               uint32_t frame_dlen,
                               dmcam_frame_t *first_frame_info);

/**
 * get one frame into specified buffer. this function is
 * non-blocking, if no frame is ready, it returns 0
 *
 * @param dev [in] dmcam device handler
 * @param frame_data [out] frame data  to be filled, it can be
 *                   NULL
 * @param frame_info [out] frame attributes. It can be NULL
 *
 * @return int return 0 if not frame is ready, else return 1
 */
__API int dmcam_cap_get_frame(dmcam_dev_t *dev, uint8_t *frame_data, uint32_t frame_dlen, dmcam_frame_t *frame_info);

/**
 * save specified raw frame data into replay file. This function
 * is only functional when @ref en_save_replay and
 * @ref en_save_manually in @ref dmcam_cap_cfg_t is set to true.
 *
 * @param dev [in] dmcam device handler
 * @param frame_data [in] raw frame data to be saved.
 * @param frame_dlen [in] raw frame data len
 * @param frame_info [in] frame info of the raw frame data
 *
 * @return __API int return 0 if saving is ok, else return
 *         negative number
 */

__API int dmcam_cap_save_frame(dmcam_dev_t *dev,
                               const uint8_t *frame_data,
                               uint32_t frame_dlen,
                               const dmcam_frame_info_t *frame_info);

/**
 * seek frame inside replay device. it only has effect on replay
 * file simulated dmcam device. it'll return -1 if dev is not a
 * replay device.
 *
 * @param dev [in] dmcam device handler
 * @param frame_cnt_offset [in] specified frames to seek
 *                         afterward or forward. negative value
 *                         = seek forward
 * @param whence [in] SEEK_SET: beginning of the replay.
 *                    SEEK_CUR: current frame of the replay.
 *                    SEEK_END: end of the replay.
 *
 * @return int return the current frame pos (0 = at the
 *         beginning, 1 = at the end of first frame). -1 =
 *         failed.
 */

__API int dmcam_cap_seek_frame(dmcam_dev_t *dev, int frame_cnt_offset, int whence);

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

/*****************************************************
 *       frame data get API section 
 *****************************************************/
/** alias for dmcam_frame_get_dist_f32  */
__API int dmcam_frame_get_distance(dmcam_dev_t *dev,
                                   float *dst,
                                   int dst_len,
                                   uint8_t *src,
                                   int src_len,
                                   const dmcam_frame_info_t *finfo);

/**
 * convert to raw distance data in uint16 from raw frame data.
 * the raw distance is calculated without any calibration and
 * pixel reorder.
 *
 * @param dev [in] specified dmcam device
 * @param dst [out] distance buffer. The unit of distance is in
 *            millimeter (uint16)
 * @param dst_len [in] distance buffer length in number of
 *                uint16
 * @param src [in] raw frame data buffer
 * @param src_len [in] raw frame data length in byte
 * @param finfo [in] raw frame information
 *
 * @return int [out] return the number for distance points in
 *         dst
 */
__API int dmcam_frame_get_dist_raw(dmcam_dev_t *dev,
                                   uint16_t *dst,
                                   int dst_len,
                                   uint8_t *src,
                                   int src_len,
                                   const dmcam_frame_info_t *finfo);
/**
 * convert to distance data to float32 from raw frame data.
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

__API int dmcam_frame_get_dist_f32(dmcam_dev_t *dev,
                                   float *dst,
                                   int dst_len,
                                   uint8_t *src,
                                   int src_len,
                                   const dmcam_frame_info_t *finfo);

/**
 * convert to distance data in uint16 from raw frame data.
 *
 * @param dev [in] specified dmcam device
 * @param dst [out] distance buffer. The unit of distance is in
 *            millimeter (uint16)
 * @param dst_len [in] distance buffer length in number of
 *                uint16
 * @param src [in] raw frame data buffer
 * @param src_len [in] raw frame data length in byte
 * @param finfo [in] raw frame information
 *
 * @return int [out] return the number for distance points in
 *         dst
 */

__API int dmcam_frame_get_dist_u16(dmcam_dev_t *dev,
                                   uint16_t *dst,
                                   int dst_len,
                                   uint8_t *src,
                                   int src_len,
                                   const dmcam_frame_info_t *finfo);

/** alias for dmcam_frame_get_gray_f32  */
__API int dmcam_frame_get_gray(dmcam_dev_t *dev,
                               float *dst,
                               int dst_len,
                               uint8_t *src,
                               int src_len,
                               const dmcam_frame_info_t *finfo);
/**
 * get gray data in float32 from raw frame data.
 *
 * @param dev [in] specified dmcam device
 * @param dst [out] gray buffer. The gray value denotes the amplitude.
 *            (float32 in [0, 2048.0) )
 * @param dst_len [in] distance buffer length in number of float
 * @param src [in] raw frame  data buffer
 * @param src_len [in] raw frame data length in byte
 * @param finfo [in] raw frame information
 *
 * @return int [out] return the number for gray points in
 *         dst
 */

__API int dmcam_frame_get_gray_f32(dmcam_dev_t *dev,
                                   float *dst,
                                   int dst_len,
                                   uint8_t *src,
                                   int src_len,
                                   const dmcam_frame_info_t *finfo);

/**
 * get gray data in uint16_t from raw frame data.
 *
 * @param dev [in] specified dmcam device
 * @param dst [out] gray buffer. The gray value denotes the amplitude.
 *            (uint16_t in [0, 2048))
 * @param dst_len [in] distance buffer length in number of
 *                uint16_t
 * @param src [in] raw frame  data buffer
 * @param src_len [in] raw frame data length in byte
 * @param finfo [in] raw frame information
 *
 * @return int [out] return the number for gray points in dst
 */
__API int dmcam_frame_get_gray_u16(dmcam_dev_t *dev,
                                   uint16_t *dst,
                                   int dst_len,
                                   uint8_t *src,
                                   int src_len,
                                   const dmcam_frame_info_t *finfo);

/**
 * get point cloud data from distance data. The distance data is
 * usually calcuated using dmcam_frame_get_dist_f32.
 *
 * @param dev [in] specified dmcam device
 * @param pcl [out] point clound buffer. each 3 element consists
 *            a (x,y,z) point, output is in (w,h,3) dimension
 *            and in meter unit. point in value (0,0,0) is
 *            invalid
 * @param pcl_len [in] point cloud float element count
 * @param dist [in] distance image data buffer. The unit of
 *             distance is meter (float)
 * @param dist_len [in] distance image data count (in
 *                 sizeof(float))
 * @param img_w [in] distance image width in pixel
 * @param img_h [in] distance image height in pixel
 * @param p_cam_param [in] user specified camera lens parameter.
 *                    if null, the internal camera parameter is
 *                    used.
 *
 * @return int [out] return number of points in point cloud
 *         buffer. Note: n points means 3*n floats. N should be
 *         img_w * img_h
 */
__API int dmcam_frame_get_pcl(dmcam_dev_t *dev,
                              float *pcl,
                              int pcl_len,
                              const float *dist,
                              int dist_len,
                              int img_w,
                              int img_h,
                              const dmcam_len_param_t *p_cam_param);

/**
 * get point cloud data from distance data. The distance data is
 * usually calcuated using dmcam_frame_get_dist_f32.
 * 
 * @param dev [in] specified dmcam device
 * @param pcl [out] point clound buffer. each 4 element consists
 *            a (x,y,z,d) point. (x,y,z) is coordinate in meter
 *            unit, d is distance in meter unit or pseudo-color.
 *            output is in (w,h,4) dimension. point in value
 *            (0,0,0) is invalid
 * @param pcl_len [in] point cloud float element count
 * @param dist [in] distance image data buffer. The unit of
 *             distance is meter (float)
 * @param dist_len [in] distance image data count (in
 *                 sizeof(float))
 * @param img_w [in] distance image width in pixel
 * @param img_h [in] distance image height in pixel
 * @param pseudo_color [in] if true, d is pseudo uint32 rgb
 *                     color value (can be retrieve by
 *                     (uint32)pcl[4*i + 3]); if false, d is
 *                     the distance in meter
 * @param pseudo_range_min_mm [in] if pseudo_color is true, this 
 *                            value decided the min distance
 *                            range value mapping to RGB pallete
 * @param pseudo_range_max_mm [in] if pseudo_color is true, this 
 *                            value decided the max distance
 *                            range value mapping to RGB pallete
 * @param p_cam_param [in] user specified camera lens parameter.
 *                    if null, the internal camera parameter is
 *                    used.
 * @return int [out] return number of points in point cloud
 *         buffer. Note: n points means 4*n floats. N should be
 *         img_w * img_h
 */
__API int dmcam_frame_get_pcl_xyzd(dmcam_dev_t *dev, float *pcl, int pcl_len,
                                       const float *dist, int dist_len, int img_w, int img_h,
                                       bool pseudo_color, float pseudo_range_min_mm, float pseudo_range_max_mm,
                                       const dmcam_len_param_t *p_cam_param);


/**
 * get point cloud data from distance data and gray(confid) data
 * .
 * @param dev [in] specified dmcam device
 * @param pcl [out] point clound buffer. each 4 element consists
 *            a (x,y,z,i) point.
 *   (x,y,z) is coordinate in meter unit, IR value can be get
 *   from i by ((uint32_t)i) 0xff . output is in (w,h,4)
 *   dimension. point with (x,y,z)=(0,0,0) is invalid
 *
 * @param pcl_len [in] point cloud float element count
 * @param dist [in] distance image data buffer. The unit of
 *             distance is meter (float)
 * @param dist_len [in] distance image data count (in
 *                 sizeof(float))
 * @param gray [in] gray image data buffer.
 * @param gray_len [in] gray image data count (in sizeof(float))
 * @param img_w [in] distance image width in pixel
 * @param img_h [in] distance image height in pixel
 * @param ir_balance [in] [-1024, 1024] -> [darkest, brightest]
 * @param p_cam_param [in] user specified camera lens parameter.
 *                    if null, the internal camera parameter is
 *                    used.
 *
 * @return int [out] return number of points in point cloud
 *         buffer. Note: n points means 4*n floats. N should be
 *         img_w * img_h
 */
__API int dmcam_frame_get_pcl_xyzi(dmcam_dev_t *dev,
                                   float *pcl,
                                   int pcl_len,
                                   const float *dist,
                                   int dist_len,
                                   const float *gray,
                                   int gray_len,
                                   int img_w,
                                   int img_h,
                                   int16_t ir_balance,
                                   const dmcam_len_param_t *p_cam_param);

/*****************************************************
 *       filter API section
 *****************************************************/

/** filter ID  */
typedef enum {
    DMCAM_FILTER_ID_LEN_CALIB,           /** lens calibration */
    DMCAM_FILTER_ID_PIXEL_CALIB,         /** pixel calibration*/
    DMCAM_FILTER_ID_DEPTH_FILTER,        /** Depth filter */
    DMCAM_FILTER_ID_RESERVED,            /** RESERVED */
    DMCAM_FILTER_ID_AMP,                 /** Amplitude filter control*/
    DMCAM_FILTER_ID_AUTO_INTG,           /** auto integration filter enable : use sat_ratio to adjust */
    DMCAM_FILTER_ID_SYNC_DELAY,          /**  sync delay */
    DMCAM_FILTER_ID_TEMP_MONITOR,        /** temperature monitor */
    DMCAM_FILTER_ID_HDR,                 /** HDR mode */
    DMCAM_FILTER_ID_OFFSET,              /**  set offset for calc distance */
    DMCAM_FILTER_ID_SPORT_MODE,          /**  set sport mode */
    DMCAM_FILTER_ID_SYS_CALIB,           /** using system calibration param */
    DMCAM_FILTER_ID_AMBIENT_LIGHT_CALIB, /** using ambient light calib calibration param */
    DMCAM_FILTER_ID_FLYNOISE,            /** fly noise filter */

    DMCAM_FILTER_ID_TEMP_CALIB,          /** Temperature calibration*/
    DMCAM_FILTER_ID_MEDIAN = DMCAM_FILTER_ID_DEPTH_FILTER, /**  MEDIAN is replaced with depth filter */
    DMCAM_FILTER_CNT,
    //-------------------
} dmcam_filter_id_e;

/** filter args  */
typedef union
{
    uint32_t raw;

    uint8_t case_idx;       /**  User Scenario index */
    uint32_t lens_id;       /**  DMCAM_FILTER_ID_LEN_CALIB parameter: length index*/
    uint16_t min_amp;       /**  DMCAM_FILTER_ID_AMP parameter: Min amplitude threshold*/
    uint16_t sat_ratio;     /**  DMCAM_FILTER_ID_AUTO_INTG parameter: saturation ratio threshold*/
    uint16_t sync_delay;    /**  DMCAM_FILTER_ID_SYNC_DELAY parameter: sync delay: 0 = random delay, 1 = specified delay
                               in ms */
    int16_t temp_threshold; /**  DMCAM_FILTER_ID_TEMP_MONITOR paramter: Temperature threshold for temperature monitor*/
    struct {
        uint16_t intg_3d;    /**  intg_3d: exposure time 0 */
        uint16_t intg_3dhdr; /**  intg_3dhdr: exposure time 1 */
    } intg;                  /**  DMCAM_FILTER_ID_HDR paramter */
    uint8_t median_ksize; /**  DMCAM_FILTER_ID_MEDIAN paramter:  DEPRECATED, please use  DMCAM_FILTER_ID_DEPTH_FILTER*/
    struct {
        int16_t offset_mm;    /**  DMCAM_FILTER_ID_OFFSET paramter : offset in mm*/
        int16_t scale_x1000;      /**  DMCAM_FILTER_ID_OFFSET paramter : scale x 1000 */
    };
    uint8_t sport_mode; /**  DMCAM_FILTER_ID_SPORT_MODE parameter: 0 = high motion mode, 1 = extrem high motion mode */
    uint16_t k_ambient_light; /**  DMCAM_FILTER_ID_AMBIENT_LIGHT_CALIB kcoeff of ambient light calibration */
    struct {
        uint8_t col_reduction; /** column binning:0 no binning, 1 by half  */
        uint8_t row_reduction; /** row binning: 0 no binning, 1 by half, 2 a quarter*/
    } binning_info;

    struct {
        uint8_t depth_filter_mode;     /** 0xF0 = filter strength controlled by depth_filter_strength
                                            Other values = filter controlled automatically */
        uint8_t depth_filter_strength; /** DMCAM_FILTER_ID_DEPTH_FILTER strength: [0, 31]*/
    };

    struct {
        uint8_t fly_noise_threshold; /** fly noise threshold 0~255*/
    };
} dmcam_filter_args_u;

/**
 *  Enable filter controller setting for raw data processing
 *
 * @param dev [in] dmcam device handler
 * @param filter_id [in]:defined in dmcam_filter_id_e to
 *              identify the filter
 * @param filter_arg [in] filter control args
 * @param reserved [in] reserved for future use. User should set
 *                 to 0
 * @return int 0 = OK, otherwise failed.
 */
__API int dmcam_filter_enable(dmcam_dev_t *dev, dmcam_filter_id_e filter_id, dmcam_filter_args_u *filter_arg, uint32_t reserved);

/**
 *  Disable filter controller setting for raw data processing
 *
 * @param dev [in] dmcam device handler
 * @param filter_id [in] defined in dmcam_filter_id_e to
 *              identify the filter
 *
 * @return int 0 = OK, otherwise failed.
 */
__API int dmcam_filter_disable(dmcam_dev_t *dev, dmcam_filter_id_e filter_id);


/*****************************************************
 *  CMAP utils section     
 *****************************************************/
typedef enum {
    DMCAM_CMAP_EPC = 0,  // EPC type colormap
    DMCAM_CMAP_HSV,
    DMCAM_CMAP_BWR,
    DMCAM_CMAP_JET,
    DMCAM_CMAP_GIST_RAINBOW,
    DMCAM_CMAP_RAINBOW,
    DMCAM_CMAP_SPECTRAL,
    DMCAM_CMAP_VIRIDIS,
    DMCAM_CMAP_INFERNO,
    DMCAM_CMAP_PLASMA,
    DMCAM_CMAP_MAGMA,
    DMCAM_CMAP_BLUES,
    DMCAM_CMAP_BUGN,
    DMCAM_CMAP_BUPU,
    DMCAM_CMAP_GNBU,
    DMCAM_CMAP_GREENS,
    DMCAM_CMAP_GREYS,
    DMCAM_CMAP_ORANGES,
    DMCAM_CMAP_ORRD,
    DMCAM_CMAP_PUBU,
    DMCAM_CMAP_PUBUGN,
    DMCAM_CMAP_PURD,
    DMCAM_CMAP_PURPLES,
    DMCAM_CMAP_RDPU,
    DMCAM_CMAP_REDS,
    DMCAM_CMAP_YLGN,
    DMCAM_CMAP_YLGNBU,
    DMCAM_CMAP_YLORBR,
    DMCAM_CMAP_YLORRD,
    DMCAM_CMAP_AFMHOT,
    DMCAM_CMAP_AUTUMN,
    DMCAM_CMAP_BONE,
    DMCAM_CMAP_COOL,
    DMCAM_CMAP_COPPER,
    DMCAM_CMAP_GIST_HEAT,
    DMCAM_CMAP_GRAY,
    DMCAM_CMAP_HOT,
    DMCAM_CMAP_PINK,
    DMCAM_CMAP_SPRING,
    DMCAM_CMAP_SUMMER,
    DMCAM_CMAP_WINTER,
    DMCAM_CMAP_BRBG,
    DMCAM_CMAP_COOLWARM,
    DMCAM_CMAP_PIYG,
    DMCAM_CMAP_PRGN,
    DMCAM_CMAP_PUOR,
    DMCAM_CMAP_RDBU,
    DMCAM_CMAP_RDGY,
    DMCAM_CMAP_RDYLBU,
    DMCAM_CMAP_RDYLGN,
    DMCAM_CMAP_SEISMIC,
    DMCAM_CMAP_GIST_EARTH,
    DMCAM_CMAP_TERRAIN,
    DMCAM_CMAP_OCEAN,
    DMCAM_CMAP_GIST_STERN,
    DMCAM_CMAP_BRG,
    DMCAM_CMAP_CMRMAP,
    DMCAM_CMAP_CUBEHELIX,
    DMCAM_CMAP_GNUPLOT,
    DMCAM_CMAP_GNUPLOT2,
    DMCAM_CMAP_GIST_NCAR,
    DMCAM_CMAP_NIPY_SPECTRAL,
    DMCAM_CMAP_FLAG,
    DMCAM_CMAP_PRISM,
    //---------------------
    DMCAM_CMAP_COUNT,
} dmcam_cmap_palette_e;

/** DMCAM color map output format definition   */
typedef enum {
    DMCAM_CMAP_OUTFMT_RGB,
    DMCAM_CMAP_OUTFMT_RGBA,
    DMCAM_CMAP_OUTFMT_BGR,
    DMCAM_CMAP_OUTFMT_BGRA,

    DMCAM_CMAP_OUTFMT_RGB32,
    DMCAM_CMAP_OUTFMT_BGR32,

    DMCAM_CMAP_OUTFMT_ARGB,
    DMCAM_CMAP_OUTFMT_ABGR,
    //---------------------
    DMCAM_CMAP_OUTFMT_COUNT,
} dmcam_cmap_outfmt_e;

typedef struct dmcam_cmap_cfg {
    dmcam_cmap_palette_e color_palette;  // color pallete index used during convertion
    uint8_t histeq_en;                   // 1 = enable histogram equalization during convertion
} dmcam_cmap_cfg_t;

/**
 * set default color palette used inside dmcam_cmap_ apis.
 *
 * @param cm [in] color palette
 *
 * @return bool true = set is ok. false = set failed
 */
__API bool dmcam_cmap_palette_set(dmcam_cmap_palette_e cm);

/**
 * get default color palette currently used.
 *
 * @return dmcam_cmap_palette_e current color palette
 */
__API dmcam_cmap_palette_e dmcam_cmap_palette_get(void);

/**
 * convert dist_f32 image (pixel in meter) to pesudo-RGB points
 * with specified pixel format
 *
 * @param dst [out] pseudo-RGB point buffer
 * @param dst_len [in] point buffer size in bytes
 * @param src [in] float points buffer
 * @param src_len [in] count of float points
 * @param outfmt [in] pixel format of the pseudo-RGB
 * @param range_min_m [in] minimum range of source point
 * @param range_max_m [in] max range of source point
 * @param cfg [in] refer dmcam_cmap_cfg_t. if NULL, default
 *            config is used.
 *
 * @return int [out] the count of pseudo RGB points
 */
__API int dmcam_cmap_dist_f32_to_RGB(uint8_t *dst,
                                     int dst_len,
                                     const float *src,
                                     int src_len,
                                     dmcam_cmap_outfmt_e outfmt,
                                     float range_min_m,
                                     float range_max_m,
                                     const dmcam_cmap_cfg_t *cfg);

/**
 * convert dist_u16 image (pixel in milimeter) to pesudo-RGB
 * points with specified pixel format
 *
 * @param dst [out] pseudo-RGB point buffer
 * @param dst_len [in] point buffer size in bytes
 * @param src [in] dist_u16 image buffer
 * @param src_len [in] count of u16 points
 * @param outfmt [in] pixel format of the pseudo-RGB
 * @param range_min_mm [in] minimum range of source point
 * @param range_max_mm [in] max range of source point
 * @param cfg [in] refer dmcam_cmap_cfg_t. if NULL, default
 *            config is used.
 *
 * @return int [out] the count of pseudo RGB points
 */
__API int dmcam_cmap_dist_u16_to_RGB(uint8_t *dst,
                                     int dst_len,
                                     const uint16_t *src,
                                     int src_len,
                                     dmcam_cmap_outfmt_e outfmt,
                                     uint16_t range_min_mm,
                                     uint16_t range_max_mm,
                                     const dmcam_cmap_cfg_t *cfg);
/**
 * convert gray_u16 image to IR image whose pixel is in [0~255]
 *
 * @param dst [out] IR image buffer
 * @param dst_len [in] IR image buffer size in bytes
 * @param src [in] gray_u16 image
 * @param src_len [in] count of u16 points in gray_u16 image
 * @param balance [in] [-1024, 1024] -> [darkest, brightest]
 *
 * @return int [out] the count of IR image
 */
__API int dmcam_cmap_gray_u16_to_IR(uint8_t *dst, int dst_len, const uint16_t *src, int src_len, int balance);

/**
 * convert gray_f32 image to IR image whose pixel is in [0~255]
 *
 * @param dst [out] IR image buffer
 * @param dst_len [in] IR image buffer size in bytes
 * @param src [in] gray_f32 image
 * @param src_len [in] count of f32 points in gray_f32 image
 * @param balance [in] [-1024, 1024] -> [darkest, brightest]
 *
 * @return int [out] the count of IR image
 */
__API int dmcam_cmap_gray_f32_to_IR(uint8_t *dst, int dst_len, const float *src, int src_len, int balance);

/*---------------------------------------------------------------------------*
 * File save/load API section
 *---------------------------------------------------------------------------*/

/**
 * open specified file and get file descriptor for
 * dmcam_frame_save_xxx apis.
 *
 * @param fname [in] specified filename
 *
 * @return int [out] file descriptor. < 0 = failed
 */

__API int dmcam_file_open(const char *fname, const char *mode);

/**
 * close specified file descriptor
 *
 * @param fd [in] specified file descriptor
 */
__API void dmcam_file_close(int fd);

typedef enum {
    DMCAM_FRAME_SAVE_FLOAT32 = 0,
    DMCAM_FRAME_SAVE_UINT32,
    DMCAM_FRAME_SAVE_UINT16,
    DMCAM_FRAME_SAVE_UINT8,

} dmcam_frame_save_fmt_t;

/**
 * save specified raw data (in uin16_t) with specified pixcel
 * width and height to file with specified saving format.
 *
 * @param fd [in] specified file handler
 * @param save_fmt [in] file saving format defined in
 *                 dmcam_frame_save_fmt_t. only followin format
 *                 is supported:
 *              DMCAM_FRAME_SAVE_UINT32
 *              DMCAM_FRAME_SAVE_UINT16
 * @param raw [in] raw data
 * @param raw_len [in] number of raw data (in count of
 *                 uint16_t)
 * @param img_w [in] dist data pixel width
 * @param img_h [in] dist data pixel height
 * @param dcs_cnt [in] dist data dcs sub-frame count
 * @param raw_tag [in] any string. if want to used by replay,
 *                specify (dmcam_t*)dev->product string here.
 * @return bool [out] true = save raw frame ok, false =
 *         fail
 */
__API bool dmcam_frame_save_raw(int fd,
                                dmcam_frame_save_fmt_t save_fmt,
                                const uint16_t *raw,
                                int raw_len,
                                int img_w,
                                int img_h,
                                int dcs_cnt,
                                const char *raw_tag);

/**
 * save specified distance data (in float32, unit: meter) with
 * specified pixcel width and height to file with specified
 * saving format.
 *
 * @param fd [in] specified file handler
 * @param save_fmt [in] file saving format defined in @
 *                 @ref dmcam_frame_save_fmt_t. only followin
 *                 format is supported:
 *              DMCAM_FRAME_SAVE_FLOAT32
 *              DMCAM_FRAME_SAVE_UINT32
 *              DMCAM_FRAME_SAVE_UINT16
 * @param dist [in] distance data (in float32, unit: meter)
 * @param dist_len [in] number of distance data (in count of
 *                 float)
 * @param img_w [in] dist data pixel width
 * @param img_h [in] dist data pixel height
 *
 * @return bool [out] true = save distance frame ok, false =
 *         fail
 */
__API bool dmcam_frame_save_distance(int fd,
                                     dmcam_frame_save_fmt_t save_fmt,
                                     const float *dist,
                                     int dist_len,
                                     int img_w,
                                     int img_h);

/**
 * save specified gray data (in float32) with specified pixcel
 * width and height to file with specified saving format.
 *
 * @param fd [in] specified file handler
 * @param save_fmt [in] file saving format defined in
 *                 @ref dmcam_frame_save_fmt_t. only followin
 *                 format is supported:
 *              DMCAM_FRAME_SAVE_UINT16
 *              DMCAM_FRAME_SAVE_UINT8
 * @param src [in] gray data (in float32)
 * @param src_len [in] number of distance data (in count of
 *                 float)
 * @param img_w [in] dist data pixel width
 * @param img_h [in] dist data pixel height
 *
 * @return bool [out] true = save distance frame ok, false =
 *         fail
 */
__API bool dmcam_frame_save_gray(int fd, dmcam_frame_save_fmt_t save_fmt, const float *src, int src_len, int img_w, int img_h);

/*!
 * load one raw frame from specified file fd.
 *
 * @param fd [in] specified data file fd. The fd related file is
 *           always saved by @ref dmcam_frame_save_raw api
 * @param dst [out] raw
 * @param dst_len [in] dst buffer length (in count of
 *                sizeof(uint16_t))
 * @param dst_w [out] raw frame pixel width
 * @param dst_h [out] raw frame pixel height
 * @param dst_dcsn [out] raw dcs cnt per frame
 * @param dst_tag [out] raw data tag string
 * @param tag_len [in] raw data tag buffer size
 *
 * @return int [out] length of loaded raw data (in count of
 *         sizeof(uint16))
 */
__API int dmcam_frame_load_raw(int fd,
                               uint16_t *dst,
                               int dst_len,
                               int *dst_w,
                               int *dst_h,
                               int *dst_dcsn,
                               char *dst_tag,
                               int dst_tag_len);

/**
 * load one distance frame from specified file fd.
 *
 * @param fd [in] specified data file fd. The fd related file is
 *           always saved by @ref dmcam_frame_save_distance api
 * @param dst [out] distance in float (unit: meter)
 * @param dst_len [in] dst buffer length (in count of
 *                sizeof(float))
 * @param dst_w [out] distance frame pixel width
 * @param dst_h [out] distance frame pixel height
 *
 * @return int [out] length of loaded distance data (in count of
 *         sizeof(float))
 */

__API int dmcam_frame_load_distance(int fd, float *dst, int dst_len, int *dst_w, int *dst_h);

/**
 * load one gray frame from specified file fd.
 *
 * @param fd [in] specified data file fd. The fd related file is
 *           always saved by @ref dmcam_frame_save_gray api
 * @param dst [out] gray in float (unit: meter)
 * @param dst_len [in] dst buffer length (in count of
 *                sizeof(float))
 * @param dst_w [out] gray frame pixel width
 * @param dst_h [out] gray frame pixel height
 *
 * @return int [out] length of loaded gray data (in count of
 *         sizeof(float))
 */

__API int dmcam_frame_load_gray(int fd, float *dst, int dst_len, int *dst_w, int *dst_h);



/*****************************************************
 *       lens section 
 *****************************************************/
typedef struct dmcam_lens_calib_cfg {
    uint8_t en_2d_calib; // 2d calib on xy
    uint8_t en_3d_calib; // 3d calib on z
    uint8_t en_dist_direct_z;  // use dist as z directly
}dmcam_lens_calib_cfg_t; 

/**
 * set config related with lens calibration.
 * 
 * @param dev [in] specified dmcam device
 * @param cfg 
 */

void dmcam_lens_calib_config_set(dmcam_dev_t *dev, dmcam_lens_calib_cfg_t *cfg);

/**
 * get config of lens calibration
 * 
 * @param dev [in] specified dmcam device
 * 
 * @return const dmcam_lens_calib_cfg_t* 
 */

const dmcam_lens_calib_cfg_t* dmcam_lens_calib_config_get(dmcam_dev_t *dev);

/**
 * get lens calibration data from calibration data
 * 
 * @param dev [in] specified dmcam device
 * @param param [out] lens paramters to be filled
 * @param id  [in] len id, defualt use 0
 * 
 * @return bool true = ok 
 */
bool dmcam_lens_param_get(dmcam_dev_t *dev, dmcam_len_param_t *param, uint8_t id);

/**
 * get lens parameter filename
 * 
 * @param dev  [in] specified dmcam device
 * @param name_buf [inout] filename buffer. cannot be null
 * @param buf_len [in] buffer len
 * @param bin_format [in] true = return binary len parameter 
 *                   filename 
 * @param with_devid [in] ture = return filename with device id 
 * 
 * @return char* [out] return len parameter filename (with path)
 */
char* dmcam_lens_param_filename(dmcam_dev_t *dev, char *name_buf, int buf_len, bool bin_format, bool with_devid); 


/**
 * apply lens calibration (if have) on specified distance image 
 * (dist in float), and output distance in camera coordinate. 
 *  
 * The behavior of this function is controlled by 
 * @ref dmcam_lens_calib_config_set.
 *    en_2d_calib: decide whether to apply 2d undistortion using
 *        camera intrinsics
 *    en_3d_calib: decide whether to apply 3d
 *      undistortion(gradient) using 3d calibration
 *      coefficients.
 * 
 * @param dev [in] specified dmcam device
 * @param dst [out] output distance in camera coordinate
 * @param dst_len [in] output distance buffer size in sizeof 
 *                float 
 * @param src [in] input distance image (usually returned by 
 *            @ref dmcam_frame_get_dist_f32
 * @param src_len [in] input distance image size in sizeof float 
 * @param finfo   [in] frame info when get the input image
 * @param p_cam_param [in]  camera intrinsics specified 
 *                    externally. if NULL, internally camera
 *                    intrinsics is used.
 * 
 * @return bool [out] true= ok 
 */
bool dmcam_lens_calib_apply_dist_f32(dmcam_dev_t *dev, float *dst, int dst_len, const float *src, int src_len,
                                     const dmcam_frame_info_t *finfo, const dmcam_len_param_t *p_cam_param);

/**
 * apply lens calibration (if have) on specified distance image 
 * (dist in uint16), and output distance in camera coordinate. 
 *  
 * The behavior of this function is controlled by 
 * @ref dmcam_lens_calib_config_set.
 *    en_2d_calib: decide whether to apply 2d undistortion using
 *        camera intrinsics
 *    en_3d_calib: decide whether to apply 3d
 *      undistortion(gradient) using 3d calibration
 *      coefficients.
 * 
 * @param dev [in] specified dmcam device
 * @param dst [out] output distance in camera coordinate
 * @param dst_len [in] output distance buffer size in sizeof 
 *                uint16 
 * @param src [in] input distance image (usually returned by 
 *            @ref dmcam_frame_get_dist_u16
 * @param src_len [in] input distance image size in sizeof 
 *                uint16
 * @param finfo   [in] frame info when get the input image
 * @param p_cam_param [in]  camera intrinsics specified 
 *                    externally. if NULL, internally camera
 *                    intrinsics is used.
 * 
 * @return bool [out] true= ok 
 */
bool dmcam_lens_calib_apply_dist_u16(dmcam_dev_t *dev, uint16_t *dst, int dst_len, const uint16_t *src, int src_len,
                                     const dmcam_frame_info_t *finfo, const dmcam_len_param_t *p_cam_param);
/**
 * apply lens calibration (if have) on specified gray 
 * (intensity) image (pixel in float), and output with lens 2d 
 * undistorted image 
 *  
 * The behavior of this function is controlled by 
 * @ref dmcam_lens_calib_config_set.
 *    en_2d_calib: decide whether to apply 2d undistortion using
 *        camera intrinsics. if false, this function just copy
 *        src to dst
 * 
 * @param dev [in] specified dmcam device
 * @param dst [out] output gray image buffer
 * @param dst_len [in] output buffer size in sizeof float 
 * @param src [in] input gray image (usually returned by 
 *            @ref dmcam_frame_get_gray_f32
 * @param src_len [in] input image size in sizeof float 
 * @param finfo   [in] frame info when get the input image
 * @param p_cam_param [in]  camera intrinsics specified 
 *                    externally. if NULL, internally camera
 *                    intrinsics is used.
 * 
 * @return bool [out] true= ok 
 */
bool dmcam_lens_calib_apply_gray_f32(dmcam_dev_t *dev, float *dst, int dst_len, const float *src, int src_len,
                                     const dmcam_frame_info_t *finfo, const dmcam_len_param_t *p_cam_param);

/**
 * apply lens calibration (if have) on specified gray 
 * (intensity) image (pixel in uint16), and output with lens 2d 
 * undistorted image 
 *  
 * The behavior of this function is controlled by 
 * @ref dmcam_lens_calib_config_set.
 *    en_2d_calib: decide whether to apply 2d undistortion using
 *        camera intrinsics. if false, this function just copy
 *        src to dst
 * 
 * @param dev [in] specified dmcam device
 * @param dst [out] output gray image buffer
 * @param dst_len [in] output buffer size in sizeof uint16 
 * @param src [in] input gray image (usually returned by 
 *            @ref dmcam_frame_get_gray_u16
 * @param src_len [in] input image size in sizeof uint16 
 * @param finfo   [in] frame info when get the input image
 * @param p_cam_param [in]  camera intrinsics specified 
 *                    externally. if NULL, internally camera
 *                    intrinsics is used.
 * 
 * @return bool [out] true= ok 
 */
bool dmcam_lens_calib_apply_gray_u16(dmcam_dev_t *dev, uint16_t *dst, int dst_len, const uint16_t *src, int src_len,
                                     const dmcam_frame_info_t *finfo, const dmcam_len_param_t *p_cam_param); 

#ifdef __cplusplus
}
#endif
#endif  // DMCAM_H

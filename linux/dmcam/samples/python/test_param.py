import sys

import dmcam


def test_param_read(device):
    print("->test specified parameters reading...")
    val_mod_freq, val_frm_format = dmcam.param_batch_get(device, [dmcam.PARAM_MOD_FREQ, dmcam.PARAM_FRAME_FORMAT])
    assert val_mod_freq is not None
    print("  val_mod_freq: %d" % val_mod_freq.mod_freq)
    print("  val_frm_format: %d" % val_frm_format.frame_format.format)

    print("\n->test batch parameter reading...")
    # alloc reading request to read every params
    params_to_read = list(range(dmcam.PARAM_ENUM_COUNT))
    params_val = dmcam.param_batch_get(device, params_to_read)  # type: list[dmcam.param_val_u]
    assert params_val is not None
    # printing each params
    print("dev_mode = %d" % params_val[dmcam.PARAM_DEV_MODE].dev_mode)
    print("mod_freq = %d" % params_val[dmcam.PARAM_MOD_FREQ].mod_freq)
    print("vendor: %s" % params_val[dmcam.PARAM_INFO_VENDOR].info_vendor)
    print("product: %s" % params_val[dmcam.PARAM_INFO_PRODUCT].info_product)
    print("max frame info: %d x %d, depth=%d, fps=%d, intg_us=%d"
          % (params_val[dmcam.PARAM_INFO_CAPABILITY].info_capability.max_frame_width,
             params_val[dmcam.PARAM_INFO_CAPABILITY].info_capability.max_frame_height,
             params_val[dmcam.PARAM_INFO_CAPABILITY].info_capability.max_frame_depth,
             params_val[dmcam.PARAM_INFO_CAPABILITY].info_capability.max_fps,
             params_val[dmcam.PARAM_INFO_CAPABILITY].info_capability.max_intg_us))
    print([hex(v) for v in params_val[dmcam.PARAM_INFO_SERIAL].info_serial.serial])
    print("version: sw:%d, hw:%d, sw2:%d, hw2:%d"
          % (params_val[dmcam.PARAM_INFO_VERSION].info_version.sw_ver,
             params_val[dmcam.PARAM_INFO_VERSION].info_version.hw_ver,
             params_val[dmcam.PARAM_INFO_VERSION].info_version.sw2_ver,
             params_val[dmcam.PARAM_INFO_VERSION].info_version.hw2_ver))
    print("frame format= %d" % params_val[dmcam.PARAM_FRAME_FORMAT].frame_format.format)
    print("fps = %d" % params_val[dmcam.PARAM_FRAME_RATE].frame_rate.fps)
    print("illum_power=%d %%" % params_val[dmcam.PARAM_ILLUM_POWER].illum_power.percent)
    print("intg = %d us" % params_val[dmcam.PARAM_INTG_TIME].intg.intg_us)
    pass

def test_param_write(device):
    print("->test specified parameters writing...")
    wparams = {
        dmcam.PARAM_DEV_MODE: dmcam.param_val_u(),
        dmcam.PARAM_FRAME_RATE: dmcam.param_val_u(),
        dmcam.PARAM_ILLUM_POWER: dmcam.param_val_u(),
    }
    wparams[dmcam.PARAM_DEV_MODE].dev_mode = 1
    wparams[dmcam.PARAM_FRAME_RATE].frame_rate.fps = 30
    wparams[dmcam.PARAM_ILLUM_POWER].illum_power.percent = 30

    ret = dmcam.param_batch_set(device, wparams)
    assert ret is True

    pass


if __name__ == '__main__':
    # ret = dmcam.param_batch_get_test(None, (dmcam.PARAM_INFO_PRODUCT, dmcam.PARAM_INFO_VENDOR))
    # sys.exit(0)
    # init the lib with default log file
    dmcam.init(None)

    # set debug level
    dmcam.log_cfg(dmcam.LOG_LEVEL_INFO,
                  dmcam.LOG_LEVEL_DEBUG, dmcam.LOG_LEVEL_NONE)
    print(" Open dmcam device ..")
    # open the first device
    # dev = dmcam.dev_open(dev_list[0])i
    dev = dmcam.dev_open(None)
    assert dev is not None

    test_param_write(dev)
    test_param_read(dev)

    print(" Close dmcam device ..")
    dmcam.dev_close(dev)
    dmcam.uninit()
    sys.exit(-1)

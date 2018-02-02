import sys
import numpy as np
import time

import dmcam


def on_frame_rdy(dev, f):
    # print("cap: idx=%d, num=%d" % (f.frame_fbpos, f.frame_count))
    # time.sleep(0.5)
    pass


def on_cap_err(dev, errnumber, errarg):
    print("caperr: %s" % dmcam.error_name(errnumber))
    if errnumber == dmcam.DMCAM_ERR_CAP_FRAME_DISCARD:
        print("   %d frame discarded" % int(errarg))
    # if return false, the capture process will be stopped
    return True


# init the lib with default log file
dmcam.init(None)
# init with specified log file
# dmcam.init("test.log")
# set debug level
dmcam.log_cfg(dmcam.LOG_LEVEL_INFO,
              dmcam.LOG_LEVEL_DEBUG, dmcam.LOG_LEVEL_NONE)

# list device (maximum 10 devices to store)
print(" Scanning dmcam device ..")
devs = dmcam.dev_list()
if devs is None:
    print(" No device found")
    sys.exit(1)

print("found %d device" % len(devs))

for i in range(len(devs)):
    print("DMCAM#%d [%03d:%03d:%03d]: VENDOR=%s, PROD=%s, SERIAL=%s"
          % (i, devs[i].usb_port_num, devs[i].usb_bus_num,
             devs[i].usb_dev_addr, devs[i].vendor,
             devs[i].product, devs[i].serial))

print(" Open dmcam device ..")
# open the first device
# dev = dmcam.dev_open(devs[0])
dev = dmcam.dev_open(None)
assert dev is not None

# print(" Config capture param ..")
# set 10 frames framebuffer
dmcam.cap_set_frame_buffer(dev, None, 10 * 320 * 240 * 4)
# dmcam.cap_set_callback_on_frame_ready(dev, on_frame_rdy)
dmcam.cap_set_callback_on_error(dev, on_cap_err)

# write to ramp mode
# dmcam.reg_batch_write(dev, dmcam.DEV_REG_TFC_DE, 0x31, np.array([0x1803], dtype="uint32"))
# regval = dmcam.reg_batch_read(dev, dmcam.DEV_REG_TFC_DE, 0x31, 1)
# print(regval)
# assert regval[1] == 0x1803

print(" Set paramters ...")
# write illumination power: 100%
pwr_percent = 100
wparams = {
    dmcam.PARAM_ILLUM_POWER: dmcam.param_val_u(),
    dmcam.PARAM_INTG_TIME: dmcam.param_val_u(),
    dmcam.PARAM_FRAME_FORMAT: dmcam.param_val_u(),
}
wparams[dmcam.PARAM_ILLUM_POWER].illum_power.percent = pwr_percent
wparams[dmcam.PARAM_INTG_TIME].intg.intg_us = 1000
wparams[dmcam.PARAM_FRAME_FORMAT].frame_format.format = 1

if not dmcam.param_batch_set(dev, wparams):
    print(" set parameter failed")

print(" Start capture ...")
dmcam.cap_start(dev)

f = bytearray(320 * 240 * 4 * 2)

print(" sampling 100 frames ...")
count = 0
run = True
while run:
    # get one frame
    finfo = dmcam.frame_t()
    ret = dmcam.cap_get_frames(dev, 1, f, finfo)
    # print("get %d frames" % ret)
    if ret > 0:
        w = finfo.frame_info.width
        h = finfo.frame_info.height

        print(" frame @ %d, %d, %dx%d" %
              (finfo.frame_info.frame_idx, finfo.frame_info.frame_size, w, h))

        dist_cnt, dist = dmcam.frame_get_distance(dev, w * h, f, finfo.frame_info)
        gray_cnt, gray = dmcam.frame_get_gray(dev, w * h, f, finfo.frame_info)
        # dist = dmcam.raw2dist(int(len(f) / 4), f)
        # gray = dmcam.raw2gray(int(len(f) / 4), f)

        count += 1
        if count >= 100:
            break

    else:
        break
    time.sleep(0.01)
    # break

# print("wait 3000ms")
# dmcam.cap_wait(dev, 3000)
print(" Stop capture ...")
dmcam.cap_stop(dev)

print(" Close dmcam device ..")
dmcam.dev_close(dev)
dmcam.uninit()
sys.exit(-1)

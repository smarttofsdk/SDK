import sys

import dmcam
import matplotlib
import numpy as np
import pygame
from pygame.locals import *

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import io

# ---- pygame staff ----
# Define the colors we will use in RGB format
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 960

show_hist = True

pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
h_text = pygame.font.SysFont('arial', 14)
screen.fill((0, 0, 0))
pygame.draw.line(screen, BLUE, [SCREEN_WIDTH >> 1, 0],
                 [SCREEN_WIDTH >> 1, SCREEN_HEIGHT - 1], 1)
pygame.draw.line(screen, BLUE, [0, SCREEN_HEIGHT >> 1],
                 [SCREEN_WIDTH - 1, SCREEN_HEIGHT >> 1], 1)
scale = 2


# ------------------------

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
# assert regval[0] == 0x1803

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
wparams[dmcam.PARAM_FRAME_FORMAT].frame_format.format = dmcam.DM_FRAME_FMT_DISTANCE
if not dmcam.param_batch_set(dev, wparams):
    print(" set parameter failed")

print(" Start capture ...")
dmcam.cap_start(dev)

f = bytearray(320 * 240 * 4 * 2)
run = True
while run:
    # get one frame
    finfo = dmcam.frame_t()
    ret = dmcam.cap_get_frames(dev, 1, f, finfo)
    # print("get %d frames" % ret)
    if ret > 0:
        print(" frame @ %d, %dx%d (%d)" %
              (finfo.frame_info.frame_idx,
               finfo.frame_info.width,
               finfo.frame_info.height,
               finfo.frame_info.frame_size))
        # print the first 16bytes of the frame
        # print([hex(n) for n in f][:16])
        w = finfo.frame_info.width
        h = finfo.frame_info.height

        gray_cnt = 0

        dist_cnt, dist = dmcam.frame_get_distance(dev, w * h, f, finfo.frame_info)
        gray_cnt, gray = dmcam.frame_get_gray(dev, w * h, f, finfo.frame_info)

        if dist_cnt != w * h:
            dist = None

        if gray_cnt != w * h:
            gray = None
        # dist = dmcam.raw2dist(int(len(f) / 8), f)
        # gray = dmcam.raw2gray(int(len(f) / 4), f)
        # amb = dmcam.raw2amb(int(len(f) / 4), f)

        # --- show in TL sub-window ---
        if gray is not None:
            Z = gray.reshape(h, w, 1)
            Z = (255.0 * Z / Z.max()).astype(int)
            # convert to 320x240x3 gray
            Z = np.concatenate((Z, Z, Z), axis=2)
            gray_surf = pygame.surfarray.make_surface(Z.swapaxes(0, 1))
            if scale != 1:
                gray_surf = pygame.transform.scale(gray_surf, (scale * 320, scale * 240))
            screen.blit(gray_surf, (0, 0))

        # -- show in TR sub-window ---
        if dist is not None:
            Z = dist.reshape(h, w, 1)
            Z = (255.0 * (Z - Z.min()) / (Z.max() - Z.min())).astype(int)
            # convert to 320x240x3 gray
            Z = np.concatenate((Z, Z, Z), axis=2)
            dist_surf = pygame.surfarray.make_surface(Z.swapaxes(0, 1))
            if scale != 1:
                dist_surf = pygame.transform.scale(dist_surf, (scale * 320, scale * 240))
            screen.blit(dist_surf, (SCREEN_WIDTH / 2, 0))

            # -- show in BL sub-window ---
            # Z = amb.reshape(240, 320, 1)
            # Z = (255.0 * Z / Z.max()).astype(int)
            # # convert to 320x240x3 gray
            # Z = np.concatenate((Z, Z, Z), axis=2)
            # dist_surf = pygame.surfarray.make_surface(Z.swapaxes(0, 1))
            # if scale != 1:
            # dist_surf = pygame.transform.scale(dist_surf, (scale * 320, scale * 240))
        # screen.blit(dist_surf, (0, SCREEN_HEIGHT >> 1))

        # -- show in BR sub-window ---
        if show_hist:
            buf = io.BytesIO()
            bin_cnt, bin_edge = np.histogram(dist, bins=20)
            plt.hist(dist.ravel(), bin_edge)
            plt.savefig(buf, format='png')
            buf.seek(0)
            hist_im = pygame.image.load(buf)
            hist_im = pygame.transform.scale(hist_im, (SCREEN_WIDTH >> 1, SCREEN_HEIGHT >> 1))
            buf.close()
            plt.clf()
            screen.blit(hist_im, (SCREEN_WIDTH >> 1, SCREEN_HEIGHT >> 1))

        # show mouse position
        x, y = pygame.mouse.get_pos()
        # label = h_text.render('point@' + str(x) + ', ' + str(y), 1, (0, 128, 255))
        # screen.blit(label, (10, 10))

        pygame.display.set_caption('point@' + str(x) + ', ' + str(y))

        # print some label
        pygame.draw.rect(screen, BLUE, [5, 5, 70, 25])
        pygame.draw.rect(screen, BLUE, [(SCREEN_WIDTH >> 1) + 5, 5, 70, 25])
        pygame.draw.rect(screen, BLUE, [5, (SCREEN_HEIGHT >> 1) + 5, 70, 25])
        pygame.draw.rect(screen, BLUE, [(SCREEN_WIDTH >> 1) + 5, (SCREEN_HEIGHT >> 1) + 5, 70, 25])

        screen.blit(h_text.render("GRAY", 1, WHITE), (10, 10))
        screen.blit(h_text.render("DISTANCE", 1, WHITE), ((SCREEN_WIDTH >> 1) + 10, 10))
        screen.blit(h_text.render("AMBIENT", 1, WHITE), (10, (SCREEN_HEIGHT >> 1) + 10))
        screen.blit(h_text.render("HIST", 1, WHITE), ((SCREEN_WIDTH >> 1) + 10, (SCREEN_HEIGHT >> 1) + 10))

        pygame.display.update()
    elif ret < 0:
        break

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            run = False

# print("wait 3000ms")
# dmcam.cap_wait(dev, 3000)
print(" Stop capture ...")
dmcam.cap_stop(dev)

print(" Close dmcam device ..")
dmcam.dev_close(dev)
dmcam.uninit()
sys.exit(-1)

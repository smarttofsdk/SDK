import sys
import os
import dmcam
import matplotlib
import numpy as np
import pygame
from pygame.locals import *

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import io

dev_uri = sys.argv[1] if len(sys.argv) >= 2 else None

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

print(" Scanning dmcam device ..")
devs = dmcam.dev_list()

if devs is None:
    print(" No device found")
else:
    print("found %d device" % len(devs))
    print(" Device URIs:")
    for i, d in enumerate(devs):
        print("[#%d]: %s" % (i, dmcam.dev_get_uri(d, 256)[0]))

if dev_uri:
    dev = dmcam.dev_open_by_uri(os.fsencode(sys.argv[1]))
else:
    # open the first device
    # dev = dmcam.dev_open(devs[0])
    dev = dmcam.dev_open(None)

if dev is None:
    print(" Open device failed")
    sys.exit(-1)
else:
    print(" Open dmcam device: %s " % dmcam.dev_get_uri(dev, 256)[0])

print(" Config capture param ..")
cap_cfg = dmcam.cap_cfg_t()
cap_cfg.cache_frames_cnt = 10  # frame buffer = 10 frames
# cap_cfg.on_frame_ready = None  # callback should be set by dmcam.cap_set_callback_on_frame_ready
# cap_cfg.on_cap_err = None      # callback should be set by dmcam.cap_set_callback_on_error
cap_cfg.en_save_dist_u16 = False  # save dist into ONI file: which can be viewed in openni
cap_cfg.en_save_gray_u16 = False  # save gray into ONI file: which can be viewed in openni
cap_cfg.en_save_replay = False  # save raw into ONI file:  which can be simulated as DMCAM device
cap_cfg.fname_replay = os.fsencode("replay_dist.oni")

dmcam.cap_config_set(dev, cap_cfg)

# dmcam.cap_set_callback_on_frame_ready(dev, on_frame_rdy)
# dmcam.cap_set_callback_on_error(dev, on_cap_err)

print(" Set paramters ...")
wparams = {
    dmcam.PARAM_FRAME_RATE: dmcam.param_val_u(),
    dmcam.PARAM_INTG_TIME: dmcam.param_val_u(),
}
wparams[dmcam.PARAM_FRAME_RATE].frame_rate.fps = 15
wparams[dmcam.PARAM_INTG_TIME].intg.intg_us = 1000

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
        print(" frame @ %d, %dx%d (%d)" % (finfo.frame_info.frame_idx, finfo.frame_info.width,
                                           finfo.frame_info.height, finfo.frame_info.frame_size))
        # print the first 16bytes of the frame
        # print([hex(n) for n in f][:16])
        w, h = (finfo.frame_info.width, finfo.frame_info.height)

        gray_cnt = 0

        dist_cnt, dist = dmcam.frame_get_dist_u16(dev, w * h, f, finfo.frame_info)
        gray_cnt, gray = dmcam.frame_get_gray(dev, w * h, f, finfo.frame_info)

        if dist_cnt != w * h:
            dist = None

        if gray_cnt != w * h:
            gray = None

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
            dist_cnt, dist_img = dmcam.cmap_dist_u16_to_RGB(len(dist) * 3, dist,
                                                            dmcam.DMCAM_CMAP_OUTFMT_RGB,
                                                            0, 4000)
            Z = dist_img.reshape(h, w, 3)
            # Z = (255.0 * (Z - Z.min()) / (Z.max() - Z.min())).astype(int)
            # convert to 320x240x3 gray
            # Z = np.concatenate((Z, Z, Z), axis=2)
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

print(" Stop capture ...")
dmcam.cap_stop(dev)

print(" Close dmcam device ..")
dmcam.dev_close(dev)
dmcam.uninit()

sys.exit(-1)

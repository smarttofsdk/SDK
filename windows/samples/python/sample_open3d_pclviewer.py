import sys, os
import numpy as np
import time
import open3d as o3d
import dmcam

# --  init the lib with default log file
dmcam.init(None)
# --  init with specified log file
# dmcam.init("test.log")

# -- set debug level
dmcam.log_cfg(dmcam.LOG_LEVEL_INFO, dmcam.LOG_LEVEL_DEBUG, dmcam.LOG_LEVEL_NONE)

# -- list device

print(" Scanning dmcam device ..")
devs = dmcam.dev_list()
if devs is None:
    print(" No device found")
    sys.exit(1)

print("found %d device" % len(devs))

for i in range(len(devs)):
    print("#%d: %s" % (i, dmcam.dev_get_uri(devs[i], 256)[0]))

print(" Open dmcam device ..")
# open the first device
dev = dmcam.dev_open(devs[0])
# Or open by URI
# dev = dmcam.dev_open_by_uri(br"xxx")
assert dev is not None

# - set capture config  -
cap_cfg = dmcam.cap_cfg_t()
cap_cfg.cache_frames_cnt = 10  # framebuffer = 10
cap_cfg.on_error = None        # use cap_set_callback_on_error to set cb
cap_cfg.on_frame_rdy = None    # use cap_set_callback_on_frame_ready to set cb
cap_cfg.en_save_replay = False  # True = save replay, False = not save
cap_cfg.en_save_dist_u16 = False # True to save dist stream for openni replay
cap_cfg.en_save_gray_u16 = False # True to save gray stream for openni replay
cap_cfg.fname_replay = os.fsencode("dm_replay.oni")  # set replay filename

dmcam.cap_config_set(dev, cap_cfg)
# dmcam.cap_set_callback_on_frame_ready(dev, on_frame_rdy)
# dmcam.cap_set_callback_on_error(dev, on_cap_err)

print(" Set paramters ...")
wparams = {
    dmcam.PARAM_INTG_TIME: dmcam.param_val_u(),
    dmcam.PARAM_FRAME_RATE: dmcam.param_val_u(),
    dmcam.PARAM_MOD_FREQ: dmcam.param_val_u(),

}

# set dual frequency modulation, note that mod_freq0 should always be larger than mod_freq1
# you can trivially set mod_freq1 to zeros, if single frequency modulation is desired 
wparams[dmcam.PARAM_MOD_FREQ].mod_freq0=100000000
wparams[dmcam.PARAM_MOD_FREQ].mod_freq1=36000000

wparams[dmcam.PARAM_INTG_TIME].intg.intg_us = 1000
wparams[dmcam.PARAM_FRAME_RATE].frame_rate.fps = 20

amp_min_val = dmcam.filter_args_u()
amp_min_val.min_amp = 80

if not dmcam.filter_enable(dev, dmcam.DMCAM_FILTER_ID_AMP, amp_min_val,
                           sys.getsizeof(amp_min_val)):
    print("set amp to %d %% failed" % amp_min_val.min_amp)

if not dmcam.param_batch_set(dev, wparams):
    print(" set parameter failed")

print(" Start capture ...")
dmcam.cap_start(dev)

f = bytearray(640 * 480 * 4 * 2)
print(" sampling 100 frames ...")
count = 0
run = True

vis = o3d.visualization.Visualizer()
vis.create_window()
opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])
opt.point_size = 1
opt.show_coordinate_frame = True
pcd = o3d.geometry.PointCloud()
bind_flag = False

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
        _, pcloud = dmcam.frame_get_pcl(dev, w * h*3, dist, w, h, None)

        

        pcd.points=o3d.utility.Vector3dVector(pcloud.reshape(-1, 3))
        if not bind_flag:
            vis.add_geometry(pcd)
            bind_flag = True
        else:
            vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        #o3d.visualization.draw_geometries([pcd])

        # dist = dmcam.raw2dist(int(len(f) / 4), f)
        # gray = dmcam.raw2gray(int(len(f) / 4), f)


    else:
        break
    time.sleep(0.01)
    # break

vis.destroy_window()

print(" Stop capture ...")
dmcam.cap_stop(dev)

print(" Close dmcam device ..")
dmcam.dev_close(dev)

dmcam.uninit()

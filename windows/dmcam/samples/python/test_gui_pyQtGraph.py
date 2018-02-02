# import cv2
import dmcam
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

# from scipy import ndimage

# map the normalized data to colors

# QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
win = pg.GraphicsWindow(title="dmcam basic example", border=True)
win.resize(800, 600)
win.setWindowTitle('dmcam QtGraph example')

# Enable anti-aliasing for prettier plots
pg.setConfigOptions(antialias=True)

l = win

# Add 2 plots into the first row (automatic position)
img_gray = pg.ImageItem(np.zeros((320, 240)))
img_dist = pg.ImageItem(np.zeros((320, 240)))
img_ambient = pg.ImageItem(np.zeros((320, 240)))

vb_dist = l.addViewBox(col=0, colspan=2, rowspan=2)
vb_gray = l.addViewBox(col=2, colspan=1, rowspan=1)
l.nextRow()
vb_ambient = l.addViewBox(col=2, colspan=1, rowspan=1)
l.nextRow()
plot_hist = l.addPlot(title="Distance Hist", rowspan=1)
plot_data = plot_hist.plot(pen='y', stepMode=True, brush=(0, 0, 255, 0))

plot_ghist = l.addPlot(title="Gray Hist", rowspan=1)
plot_ghist_data = plot_ghist.plot(pen='y', stepMode=True, brush=(0, 255, 0, 150))

plot3 = l.addPlot(title="N/A", rowspan=1)

vb_gray.addItem(img_gray)
vb_gray.autoRange()

vb_dist.addItem(img_dist)
vb_dist.autoRange()

vb_ambient.addItem(img_ambient)
vb_ambient.autoRange()

# mutex for exchange frame_data
frame_data = bytearray(320 * 240 * 4 * 4)
f_dist = None
f_gray = None
dev = None
f_mutex = QtCore.QMutex()

# -----------  dmcam data sampler ------------


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

print(" Open dmcam device ..")
# open the first device
# dev = dmcam.dev_open(dev_list[0])
dev = dmcam.dev_open(None)
assert dev is not None

# print(" Config capture param ..")
# set 10 frames framebuffer
dmcam.cap_set_frame_buffer(dev, None, 10 * 320 * 240 * 4 * 2)
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
wparams[dmcam.PARAM_INTG_TIME].intg.intg_us = 500
wparams[dmcam.PARAM_FRAME_FORMAT].frame_format.format = dmcam.DM_FRAME_FMT_DISTANCE
if not dmcam.param_batch_set(dev, wparams):
    print(" set parameter failed")


class DmcamThread(QtCore.QThread):
    # Signals to relay thread progress to the main GUI thread
    frameReadySignal = QtCore.Signal(int)
    captureDoneSignal = QtCore.Signal(int)
    run = True

    def __init__(self, parent=None):
        super(DmcamThread, self).__init__(parent)
        # You can change variables defined here after initialization - but before calling start()

    def stop_me(self):
        self.run = False

    def run(self):
        print(" Start capture ...")
        dmcam.cap_start(dev)
        # blocking code goes here
        self.run = True
        while self.run:
            global f_dist, f_gray, frame_data, f_mutex
            # get one frame
            finfo = dmcam.frame_t()
            ret = dmcam.cap_get_frames(dev, 1, frame_data, finfo)  # will blocking wait
            # ret = dmcam.cap_get_frame(dev, frame_data, None)
            # print("get %d frames" % ret)
            if ret > 0:
                w = finfo.frame_info.width
                h = finfo.frame_info.height
                # print(" frame @ %d, %dx%d (%d)" %
                #       (finfo.frame_info.frame_idx,
                #        finfo.frame_info.width,
                #        finfo.frame_info.height,
                #        finfo.frame_info.frame_size))

                f_mutex.lock()
                dist_cnt, f_dist = dmcam.frame_get_distance(dev, w * h, frame_data, finfo.frame_info)
                gray_cnt, f_gray = dmcam.frame_get_gray(dev, w * h, frame_data, finfo.frame_info)
                if dist_cnt != w * h:
                    f_dist = None

                if gray_cnt != w * h:
                    f_gray = None
                f_mutex.unlock()
                self.frameReadySignal.emit(0)

        self.captureDoneSignal.emit(int(self.run))


def handle_capture_done():
    global dev
    print(" Stop capture ...")
    dmcam.cap_stop(dev)

    print(" Close dmcam device ..")
    dmcam.dev_close(dev)
    dmcam.uninit()


np_dist_buf = []


def handle_frame_ready():
    # print("ready...")
    f_mutex.lock()
    np_dist = f_dist.reshape(240, 320) if f_dist is not None else None
    np_gray = f_gray.reshape(240, 320) if f_gray is not None else None
    f_mutex.unlock()
    # np_dist = np.flipud(np.fliplr(np_dist))
    # np_gray = np.flipud(np.fliplr(np_gray))
    # np_amb = np.flipud(np.fliplr(np_amb))

    if np_dist is not None:
        # -- correct value
        np_dist = np_dist - 2.0
        # -- show dist
        # np_dist = (255.0 * np_dist / np_dist.max()).astype(int)
        # convert to 320x240x3 gray
        # np_dist = ndimage.gaussian_filter(np_dist, 0.5)
        # np_dist = ndimage.median_filter(np_dist, 3)
        # filter dist using guide filter
        # np_dist = cv2.ximgproc.guidedFilter(np_gray, np_dist, 1, 1)

        # if len(np_dist_buf) < 3:
        #     np_dist_buf.append(np_dist)
        # else:
        #     np_dist_buf[0] = np_dist_buf[1]
        #     np_dist_buf[1] = np_dist_buf[2]
        #     np_dist_buf[2] = np.copy(np_dist)
        #     np_dist = (np_dist + np_dist_buf[0] + np_dist_buf[1]) / 3
        # imgs = np.dstack(np_dist_buf)
        # np_dist = np.median(imgs, axis=2)

        # np_dist_mask = (np_dist < 10).astype(int)
        # np_dist = np_dist * np_dist_mask  # limit to 10 meter region

        # np_dist = np.concatenate((np_dist, np_dist, np_dist), axis=2)
        # norm = plt.Normalize(vmin=np_dist.min(), vmax=np_dist.max())
        norm = plt.Normalize(vmin=0, vmax=5)
        # a colormap and a normalization instance
        cmap = plt.get_cmap("hsv")  # seismic
        #np_dist_cmap = cmap(norm(np_dist))
        m = cm.ScalarMappable(norm=norm, cmap=cmap)
        np_dist_cmap = m.to_rgba(np_dist)
        
        img_dist.setImage(np_dist_cmap.swapaxes(1, 0), autoLevels=True, autoDownsample=True)
        vb_dist.autoRange()

        # -- show hist
        y, x = np.histogram(np_dist.ravel(), bins=np.linspace(0.02, 5, 100))
        plot_data.setData(x, y)

    if np_gray is not None:
        # -- scale to (0,255)
        np_gray = (65535.0 * (np_gray - np_gray.min()) / (np_gray.max() - np_gray.min()) + 0.5).astype(int)
        # -- Histograms Equalization
        hist,bins = np.histogram(np_gray.flatten(),65536,[0,65536])
        cdf = hist.cumsum()

        cdf_m = np.ma.masked_equal(cdf,0)
        cdf_m = (cdf_m - cdf_m.min())*255/(cdf_m.max()-cdf_m.min())
        cdf = np.ma.filled(cdf_m,0).astype('uint8')
        np_gray_equ = cdf[np_gray]        
        
        # convert to 320x240x3 gray
        np_gray = np.repeat(np_gray_equ, 3).reshape(np_gray_equ.shape[0], np_gray_equ.shape[1],-1)
        img_gray.setImage(np_gray.swapaxes(1, 0), autoLevels=True, autoDownsample=True, cmap="gray")
        vb_gray.autoRange()

        y, x = np.histogram(np_gray, bins=100)
        plot_ghist_data.setData(x, y)

        # -- show amb
        # np_amb = (255.0 * np_amb / np_amb.max()).astype(int)
        # convert to 320x240x3 amb
        # np_amb = np.concatenate((np_amb, np_amb, np_amb), axis=2)
        # img_ambient.setImage(np_amb, autoLevels=True, autoDownsample=True)
        # vb_ambient.autoRange()


timer = QtCore.QTimer()
cap_thread = DmcamThread()


def start_capture():
    timer.stop()
    # --- start my thread ---
    if not cap_thread.isRunning():
        cap_thread.captureDoneSignal.connect(handle_capture_done)
        cap_thread.frameReadySignal.connect(handle_frame_ready)

        cap_thread.start()
        cap_thread.setPriority(QtCore.QThread.LowPriority)


timer.timeout.connect(start_capture)
timer.start(500)

# Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        handle_capture_done()

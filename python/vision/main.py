#!/usr/bin/env python
import numpy as np
import cv2
import threading
import matplotlib.pyplot as plt
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel


try:
    from pylibfreenect2 import OpenCLPacketPipeline
    pipeline = OpenCLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenGLPacketPipeline
        pipeline = OpenGLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)

# Create and set logger
logger = createConsoleLogger(LoggerLevel.Debug)
setGlobalLogger(logger)

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

device.start()

# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

# Optinal parameters for registration
# set True if you need
need_bigdepth = False
need_color_depth_map = False

bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
color_depth_map = np.zeros((424, 512),  np.int32).ravel() \
    if need_color_depth_map else None

SIZE_AVG_FILTER = 5

class VideoStreamer:
    def __init__(self, live=False):
        plt.ion()

        frames = listener.waitForNewFrame()

        frame = frames["color"]
        ir = frames["ir"]
        depth = frames["depth"]

        registration.apply(frame, depth, undistorted, registered,
                       bigdepth=bigdepth,
                       color_depth_map=color_depth_map)   
        cap = cv2.VideoCapture(1)
        print(cv2.CAP_OPENNI_ASUS)
        _, frame = cap.read()

        #self.dim = frame.shape
        #self.frame_buffer = np.zeros((SIZE_AVG_FILTER,) + self.dim)
        #self.lock = threading.Lock()
        #self.is_live = live
        #self.run = True

        self.video_stream = threading.Thread(target=self.stream_video_thread, args=(cap,))
        self.video_stream.start()

    def imshow(self, img, name="Image"):
        cv2.imshow(name, img)
        if not self.is_live:
            cv2.waitKey(0)

    def stream_video_thread(self, cap):
        self.is_live = True

        idx = 0
        while self.run:
            _, frame = cap.read()
            self.lock.acquire()
            self.frame_buffer[idx,:,:,:] = frame
            self.lock.release()

            idx += 1
            if idx >= SIZE_AVG_FILTER:
                idx = 0

    def capture_video(self):
        if self.is_live:
            while True:
                self.lock.acquire()
                frame = self.frame_buffer.mean(axis=0).round().astype(np.uint8)
                self.lock.release()

                cv2.imshow("Video", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('x'):
                    self.run = False
                    break
                elif key == ord(' '):
                    cv2.imwrite("frame.png", frame)

                yield frame
        else:
            yield cv2.imread("frame.png")

def filter_red(app, img_rgb):
    img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)
    # imshow(np.hstack((img_hsv[:,:,0], img_hsv[:,:,1], img_hsv[:,:,2])), "HSV")
    # plt.clf()
    # plt.imshow(np.hstack((img_hsv[:,:,0], img_hsv[:,:,1], img_hsv[:,:,2])), cmap="nipy_spectral")
    # plt.colorbar()
    # plt.pause(0.01)

    mask_fg = np.maximum(cv2.inRange(img_hsv, np.array([130,50,0]), np.array([180,255,255])),
                      cv2.inRange(img_hsv, np.array([0  ,50,0]), np.array([30 ,255,255])))
    mask_fg = cv2.morphologyEx(mask_fg, cv2.MORPH_OPEN, np.ones((10,10)))
    mask_bg = cv2.bitwise_not(mask_fg)
    mask_bg = cv2.morphologyEx(mask_bg, cv2.MORPH_ERODE, np.ones((100,100)))
    mask_fg = cv2.morphologyEx(mask_fg, cv2.MORPH_ERODE, np.ones((30,30)))
    app.imshow(mask_fg, "Mask")
    app.imshow(mask_bg, "Background Mask")

    markers = np.zeros(mask_fg.shape, dtype=np.int32)
    markers[mask_fg > 0] = 1
    markers[mask_bg > 0] = 2
    markers = cv2.watershed(img_hsv, markers)
    mask_red = (255 * (markers==1)).astype(np.uint8)

    # img_red = img_rgb.copy()
    # moments = cv2.moments(mask, True)
    # seed = np.array([moments["m10"]/moments["m00"], moments["m01"]/moments["m00"]]).round().astype(np.uint8)
    # cv2.floodFill(img_red, np.zeros((mask.shape[0]+2, mask.shape[1]+2), dtype=np.uint8), (seed[0], seed[1]), 0)
    # imshow(img_red)

    img_hsv = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_red)
    img_rgb2 = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
    return img_rgb2

if __name__ == "__main__":
    app = VideoStreamer(live=True)

    for img_rgb in app.capture_video():
        img_hsv = filter_red(app, img_rgb)
        app.imshow(img_hsv, "Filtered")


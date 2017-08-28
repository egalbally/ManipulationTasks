# coding: utf-8

import numpy as np
import cv2
import sys
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
need_bigdepth = True
need_color_depth_map = True

bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
color_depth_map = np.zeros((424, 512),  np.int32).ravel() \
    if need_color_depth_map else None

while True:
    frames = listener.waitForNewFrame()

    color = frames["color"]
    ir = frames["ir"]
    depth = frames["depth"]

    registration.apply(color, depth, undistorted, registered,
                       bigdepth=bigdepth,
                       color_depth_map=color_depth_map)

    # NOTE for visualization:
    # cv2.imshow without OpenGL backend seems to be quite slow to draw all
    # things below. Try commenting out some imshow if you don't have a fast
    # visualization backend.
    #cv2.imshow("ir", ir.asarray() / 65535.)
    cv2.imshow("depth", depth.asarray() / 4500.)
    cv2.imshow("color", cv2.resize(color.asarray(),
                                   (int(1920 / 3), int(1080 / 3))))
    cv2.imshow("registered", registered.asarray(np.uint8))

    if need_bigdepth:
        cv2.imshow("bigdepth", cv2.resize(bigdepth.asarray(np.float32),
                                          (int(1920 / 3), int(1082 / 3))))
    if need_color_depth_map:
        cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))

    
    img_hsv = cv2.cvtColor(color.asarray(), cv2.COLOR_BGR2HSV)
    # imshow(np.hstack((img_hsv[:,:,0], img_hsv[:,:,1], img_hsv[:,:,2])), "HSV")
    # plt.clf()
    # plt.imshow(np.hstack((img_hsv[:,:,0], img_hsv[:,:,1], img_hsv[:,:,2])), cmap="nipy_spectral")
    # plt.colorbar()
    # plt.pause(0.01)

    mask_fg = np.maximum(cv2.inRange(img_hsv, np.array([160,50,0]), np.array([180,255,255])),
                      cv2.inRange(img_hsv, np.array([0  ,50,0]), np.array([10 ,255,255])))
    mask_fg = cv2.morphologyEx(mask_fg, cv2.MORPH_OPEN, np.ones((10,10)))
    mask_bg = cv2.bitwise_not(mask_fg)
    mask_bg = cv2.morphologyEx(mask_bg, cv2.MORPH_ERODE, np.ones((100,100)))
    mask_fg = cv2.morphologyEx(mask_fg, cv2.MORPH_ERODE, np.ones((30,30)))

    cv2.imshow("Mask", mask_fg)
    cv2.imshow("Background Mask", mask_bg)

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

    cv2.imshow( "Filtered", img_rgb2)

    listener.release(frames)

    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

device.stop()
device.close()

sys.exit(0)
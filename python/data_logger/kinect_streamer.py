#!/usr/bin/env python

from argparse import ArgumentParser
import numpy as np
import cv2
import sys
import threading
import time
import signal
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

FPS = 30.0
RES_COLOR = (1920, 1080)
RES_DEPTH = (512, 424)
SIZE_AVG_FILTER = 1

class KinectStreamer:

    def __init__(self):
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

        self.freenect = Freenect2()
        num_devices = self.freenect.enumerateDevices()
        if num_devices == 0:
            print("No device connected!")
            sys.exit(1)

        serial = self.freenect.getDeviceSerialNumber(0)
        self.device = self.freenect.openDevice(serial, pipeline=pipeline)

        self.listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)

        # Register listeners
        self.device.setColorFrameListener(self.listener)
        self.device.setIrAndDepthFrameListener(self.listener)

        self.device.start()

        self.registration = Registration(self.device.getIrCameraParams(),
                                         self.device.getColorCameraParams())

        self.run_loop = True
        self.lock = threading.Lock()
        self.img_buffer = np.zeros((RES_COLOR[1], RES_COLOR[0], 3, SIZE_AVG_FILTER))
        self.idx_buffer = 0

        # Close on Ctrl-C
        def signal_handler(signal, frame):
            self.run_loop = False
        signal.signal(signal.SIGINT, signal_handler)

    def stream_thread(self):
        undistorted = Frame(RES_DEPTH[0], RES_DEPTH[1], 4)
        registered = Frame(RES_DEPTH[0], RES_DEPTH[1], 4)

        while self.run_loop:
            frames = self.listener.waitForNewFrame()

            color = frames["color"]
            ir = frames["ir"]
            depth = frames["depth"]

            self.registration.apply(color, depth, undistorted, registered)

            img_ir = ir.asarray() / 65535.
            self.lock.acquire()
            self.img_depth = (depth.asarray() / 4500. * 255).astype(np.uint8)
            self.img_color = np.copy(color.asarray()[:,::-1,:3])
            self.img_buffer[:,:,:,self.idx_buffer] = self.img_color
            self.img_registered_color = np.copy(registered.asarray(np.uint8))
            self.img_registered_depth = np.copy(undistorted.asarray(np.uint8))
            self.lock.release()
            self.idx_buffer += 1
            if self.idx_buffer >= SIZE_AVG_FILTER:
                self.idx_buffer = 0

            cv2.imshow("Kinect Depth", self.img_depth)
            cv2.imshow("Kinect Color", cv2.resize(self.img_color, (int(round(0.3*RES_COLOR[0])), int(round(0.3*RES_COLOR[1])))))
            cv2.imshow("Kinect R Depth", self.img_registered_depth)
            cv2.imshow("Kinect R Color", self.img_registered_color)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('x'):
                self.run_loop = False
                break
            elif key == ord(' '):
                cv2.imwrite("frame.png", frame)

            self.listener.release(frames)

        self.device.stop()
        self.device.close()

    def write_thread(self, video_name="output"):
        fourcc = cv2.VideoWriter_fourcc(*'X264')
        vid_color = cv2.VideoWriter(video_name + ".avi", fourcc, FPS, RES_COLOR)
        vid_depth = cv2.VideoWriter(video_name + "_depth.avi", fourcc, FPS, RES_DEPTH, isColor=False)

        t_curr = time.time()
        while self.run_loop:
            t_start = t_curr
            self.lock.acquire()
            try:
                vid_color.write(self.img_color)
                vid_depth.write(self.img_depth)
            except:
                pass
            self.lock.release()
            t_curr = time.time()
            time.sleep(max(1./FPS - (t_curr - t_start), 0))

        vid_color.release()
        vid_depth.release()

if __name__ == "__main__":
    # Parse arguments
    parser = ArgumentParser(description=(
        "Plot Redis values in real time."
    ))
    parser.add_argument("-rh", "--redis_host", help="Redis hostname (default: localhost)", default="localhost")
    parser.add_argument("-rp", "--redis_port", help="Redis port (default: 6379)", default=6379, type=int)
    parser.add_argument("-o", "--output", help="Output log (default: output.log)", default="output.log")
    args = parser.parse_args()

    # Initialize class
    ks = KinectStreamer()

    # Start Redis thread
    t1 = threading.Thread(target=ks.stream_thread)
    t1.daemon = True
    t1.start()

    # Start plotting thread
    ks.write_thread()

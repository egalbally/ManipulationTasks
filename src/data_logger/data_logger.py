#!/usr/bin/env python

from kinect_streamer import KinectStreamer
from realtime_plotter import RealtimePlotter
from argparse import ArgumentParser
import threading
import os
import time

DATA_DIR = "data"

if __name__ == "__main__":
    # Parse arguments
    parser = ArgumentParser(description=(
        "Plot Redis values in real time."
    ))
    parser.add_argument("-rh", "--redis_host", help="Redis hostname (default: localhost)", default="localhost")
    parser.add_argument("-rp", "--redis_port", help="Redis port (default: 6379)", default=6379, type=int)
    parser.add_argument("-o", "--output", help="Output log (default: output.log)", default=time.strftime("%m-%d-%y_%H-%M-%S.log"))
    args = parser.parse_args()

    # Prepare file names
    output_name = args.output.replace(".log","")
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)

    # Initialize class
    rp = RealtimePlotter()

    # Start Redis thread
    rp_t1 = threading.Thread(target=rp.redis_thread, args=(os.path.join(DATA_DIR, output_name + ".log"), args.redis_host, args.redis_port))
    rp_t1.daemon = True
    rp_t1.start()

    # Initialize class
    ks = KinectStreamer()

    # Start stream thread
    ks_t1 = threading.Thread(target=ks.stream_thread)
    ks_t1.daemon = True
    ks_t1.start()

    # Start write thread
    ks_t2 = threading.Thread(target=ks.write_thread, args=(os.path.join(DATA_DIR, output_name),))
    ks_t2.daemon = True
    ks_t2.start()

    # Start plotting thread
    rp.plot_thread()
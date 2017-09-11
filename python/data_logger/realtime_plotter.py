#!/usr/bin/env python

from argparse import ArgumentParser
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import redis

import threading
import time
import json

import signal
import sys

# Redis key to monitor
# REDIS_KEY = "sai2::optoforce_6d::force"
REDIS_KEY = "sai2::optoforce_6d::force"
# REDIS_KEY = "cs225a::kuka_iiwa::actuators::fgc"

# Legend labels
LABELS = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
# LABELS = ["F1", "F2", "F3", "F4", "F5", "F6", "F7"]

# Line colors
COLORS = ["r", "g", "b", "r", "g", "b"]
# COLORS = ["r", "y", "g", "c", "b", "m", "k"]

# Split data into subplots at these start indices
SUBPLOT_START = [0, 3]
# SUBPLOT_START = [0]

# Number of seconds to display
TIME_WINDOW = 10

# Y axis limits
Y_LIM = [[-10, 10], [-1, 1]]
# Y_LIM = [-10, 10]


class RealtimePlotter:

    INITIAL_WINDOW_SIZE = 1000

    def __init__(self):
        self.idx  = 0
        self.idx_lock = threading.Lock()
        self.channel = 0
        self.channel_lock = threading.Lock()
        self.size_window = RealtimePlotter.INITIAL_WINDOW_SIZE
        self.time = [np.zeros((self.size_window,)) for _ in range(2)]
        self.data = [np.zeros((len(LABELS), self.size_window)) for _ in range(2)]
        self.idx_end = [self.size_window for _ in range(2)]
        self.run_loop = True

    def redis_thread(self, logfile="output.log", host="localhost", port=6379):
        # Connect to Redis
        redis_client = redis.StrictRedis(host=host, port=port)

        # Open log file
        with open(logfile, "w") as f:
            t_init = time.time()
            t_loop = t_init
            t_elapsed = 0
            while self.run_loop:
                # Get Redis key
                str_data = redis_client.get(REDIS_KEY)
                t_curr = time.time()

                # Parse Redis string
                try:
                    if str_data[0] == "[":
                        data = json.loads(str_data)
                    else:
                        data = [float(el.strip()) for el in str_data.split(" ")]
                except:
                    print("Invalid Redis key: {0} = {1}".format(REDIS_KEY, str_data))
                    time.sleep(1.0)
                    continue

                # Write to log
                f.write("{0}\t{1}\n".format(t_curr - t_init, str_data))

                if t_curr - t_loop > TIME_WINDOW:
                    t_loop = t_curr
                    self.idx_lock.acquire()
                    self.idx_end[self.channel] = self.idx
                    self.idx = 0
                    self.idx_lock.release()
                    t_elapsed += TIME_WINDOW
                    # print("{0}s elapsed: {1} iterations/loop, {2} Hz".format(t_elapsed, self.idx_end[self.channel], float(self.idx_end[self.channel]) / TIME_WINDOW))

                    self.channel_lock.acquire()
                    self.channel = 1 - self.channel
                    self.channel_lock.release()

                # Update data
                self.time[self.channel][self.idx] = t_curr - t_loop
                self.data[self.channel][:,self.idx] = data

                # Reserve more space if idx reaches window_size
                if self.idx >= self.size_window - 1:
                    self.channel_lock.acquire()
                    for i in range(2):
                        self.time[i] = np.hstack((self.time[i], np.zeros(self.time[i].shape)))
                        self.data[i] = np.hstack((self.data[i], np.zeros(self.data[i].shape)))
                    self.channel_lock.release()
                    self.size_window *= 2

                # Increment loop index
                self.idx_lock.acquire()
                self.idx += 1
                self.idx_lock.release()

    def plot_thread(self):
        # Set up plot
        subplots = SUBPLOT_START + [len(LABELS)]
        num_subplots = len(SUBPLOT_START)
        fig, axes = plt.subplots(nrows=num_subplots)
        if num_subplots == 1:
            axes = [axes]
        lines = []
        # Add lines for current channel
        for i in range(num_subplots):
            lines += [axes[i].plot([], [], COLORS[j], label=LABELS[j], animated=True)[0] for j in range(subplots[i],subplots[i+1])]
        # Add lines for old channel
        for i in range(num_subplots):
            lines += [axes[i].plot([], [], COLORS[j] + ":", animated=True)[0] for j in range(subplots[i],subplots[i+1])]
        for ax, ylim in zip(axes, Y_LIM):
            ax.legend()
            ax.set_xlim([0, TIME_WINDOW])
            ax.set_ylim(ylim)

        # Set up animation
        t_init = time.time()
        def animate(idx):
            # Prevent redis_thread from changing channels or reallocating during this function
            self.channel_lock.acquire()

            # Find the current timestamp in the old channel
            old_channel = 1 - self.channel
            self.idx_lock.acquire()
            idx_curr = self.idx
            idx_old_end = self.idx_end[old_channel]
            self.idx_lock.release()
            t_curr = self.time[self.channel][idx_curr-1]
            idx_old_start = np.searchsorted(self.time[old_channel][:idx_old_end], t_curr, side="right")

            for i, line in enumerate(lines):
                if i < len(LABELS):
                    # Plot the current channel up to the current timestamp
                    line.set_data(self.time[self.channel][:idx_curr], self.data[self.channel][i,:idx_curr])
                else:
                    # Plot the old channel from the current timestamp
                    line.set_data(self.time[old_channel][idx_old_start:idx_old_end], self.data[old_channel][i-len(LABELS),idx_old_start:idx_old_end])

            self.channel_lock.release()
            return lines

        # Plot
        ani = FuncAnimation(fig, animate, interval=1, blit=True)

        # Close on Ctrl-C
        def signal_handler(signal, frame):
            self.run_loop = False
            plt.close()
        signal.signal(signal.SIGINT, signal_handler)

        plt.show()

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
    rp = RealtimePlotter()

    # Start Redis thread
    t1 = threading.Thread(target=rp.redis_thread, args=(args.output, args.redis_host, args.redis_port))
    t1.daemon = True
    t1.start()

    # Start plotting thread
    rp.plot_thread()

#!/usr/bin/env python

import sys

sys.path.append('/usr/local/python/openpose')
from openpose import *


class OpenPoseRunner:
    """
    Provides runner for OpenPose library.
    """

    def __init__(self, config, stream, callback = None):
        """
        :param config: configuration for OpenPose and OpenPoseRunner
        :param stream: stream of type VideoStream
        :param callback: optional callback which is called every time a frame was analyzed. Provides two values for the callback function, keypoints and image including drawn skeleton.
        """
        self.config = config
        self.running = False
        self.keypoints = []
        self.image = None
        self.openpose = OpenPose(config)
        self.stream = stream
        self.callbacks = []
        self.add_callback(callback)

    def start(self):
        self.stream.start()
        self.running = True
        self.update()
        return self

    def stop(self):
        self.stream.stop()
        self.running = False

    def is_running(self):
        return self.running

    def add_callback(self, callback):
        if callback is not None:
            self.callbacks.append(callback)

    def calculate_frame(self):
        frame = self.stream.read()
        self.keypoints, self.image = self.openpose.forward(frame, True)

    def update(self):
        while True:
            if not self.is_running():
                return
            self.calculate_frame()
            for callback in self.callbacks:
                callback(self.keypoints, self.image)

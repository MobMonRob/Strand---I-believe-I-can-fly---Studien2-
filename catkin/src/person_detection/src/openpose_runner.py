#!/usr/bin/env python

import sys

sys.path.append('/usr/local/python/openpose')
from openpose import *


# Possible performance improvements:
# TODO: Process frame in OpenPose only if frame changed from last time
# TODO: Run processing in a new thread to free up the main thread.
class OpenPoseRunner:
    """
    Provides runner for OpenPose library.
    """

    def __init__(self, config, stream, callback = None):
        """
        :param config: configuration for OpenPose and OpenPoseRunner
        :param stream: stream of type VideoStream
        :param callback: optional callback which is called every time a frame was analyzed. Provides two values for the
            callback function, keypoints and image including drawn skeleton.
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
        """
        Starts the frame processing procedure by starting the actual video capturing and starting the update loop.
        """
        self.stream.start()
        self.running = True
        self.update()

    def stop(self):
        """
        Stops the frame processing procedure by stopping the video stream and setting the internal 'running' flag to
        False.
        """
        self.stream.stop()
        self.running = False

    def is_running(self):
        """
        Returns whether frame processing is currently running.
        :return: True if frame processor is running, otherwise False
        """
        return self.running

    def add_callback(self, callback):
        """
        Appends given callback function to callback list. Callbacks are called as soon as a new frame is processed.
        :param callback: callback to be called when new frame is processed
        """
        if callback is not None:
            self.callbacks.append(callback)

    def calculate_frame(self):
        """
        Reads a single frame from the video stream and forwards it the OpenPose software. Saves the keypoints and frame
        containing the skeleton overlay afterwards.
        """
        frame = self.stream.read()
        self.keypoints, self.image = self.openpose.forward(frame, True)

    def update(self):
        """
        Starts main loop for image processing which processes a new frame every loop and calls back every registered
        callback.
        """
        while True:
            if not self.is_running():
                return
            self.calculate_frame()
            for callback in self.callbacks:
                callback(self.keypoints, self.image)

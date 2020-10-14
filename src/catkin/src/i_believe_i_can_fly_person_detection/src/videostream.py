#!/usr/bin/env python

import cv2
from threading import Thread


class VideoStream:
    """
    Provides multi-threaded video capturing to improve performance as video capturing of a camera is a thread blocking
    process which should not be performed in the Python main thread.
    """

    def __init__(self, src = 0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        """
        Starts actual capturing in a separate thread.
        """
        Thread(target = self.update, args = ()).start()

    def update(self):
        """
        Reads frames of the stream and saves it so it can be accessed later on. Infinite loop is fine as the method
        is only executed on a parallel thread and does not block the main thread.
        """
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        """
        Returns the last captured frame.
        :return: last captured frame
        """
        return self.frame

    def stop(self):
        """
        Signals the stream to stop capturing new frames.
        :return:
        """
        self.stopped = True

    def is_stopped(self):
        """
        Returns whether video stream is stopped.
        :return: True if stream is stopped, otherwise False
        """
        return self.stopped

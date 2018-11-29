#!/usr/bin/env python

import cv2
from threading import Thread


class VideoStream:
    """
    Provides multi-threaded video capturing to improve performance as video capturing of a camera is a blocking process.
    """

    def __init__(self, src = 0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target = self.update, args = ()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

    def is_stopped(self):
        return self.stopped

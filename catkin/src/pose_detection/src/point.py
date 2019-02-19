#!/usr/bin/env python

import math


class Point:
    def __init__(self, x, y, acc, index, desc):
        self.x = x
        self.y = y
        self.acc = acc
        self.index = index
        self.desc = desc

    def __str__(self):
        return "[x: " + str(self.x) + ", y: " + str(self.y) + ", acc: " + str(
            int(self.acc * 100)) + "%" + ", index: " + str(self.index) + ", desc: " + str(self.desc) + "]"

    def __repr__(self):
        return self.__str__()

    def distance_to(self, point):
        return math.sqrt((float(self.x - point.x) ** 2) + (float(self.y - point.y) ** 2))

    def gradient_to(self, point):
        # division by 0
        if self.x == point.x:
            return None
        return float(point.y - self.y) / float(point.x - self.x)

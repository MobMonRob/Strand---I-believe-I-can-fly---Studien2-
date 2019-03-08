#!/usr/bin/env python

import math


class Point:
    """
    Used as a representation for a skeletons keypoint.
    """

    def __init__(self, x, y, acc, index, desc):
        self.x = x
        self.y = y
        self.acc = acc
        self.index = index
        self.desc = desc

    def __str__(self):
        return '[x: {}, y: {}, acc: {}%" + ", index: {}, desc: {}]'.format(self.x, self.y, int(self.acc * 100),
                                                                           self.index, self.desc)

    def __repr__(self):
        return self.__str__()

    def distance_to(self, point):
        """
        Calculates the distance to any given point.
        :param point: point which the distance should be calculated to
        :return: distance to given point
        """
        return math.sqrt((float(self.x - point.x) ** 2) + (float(self.y - point.y) ** 2))

    def gradient_to(self, point):
        """
        Calculates the gradient to any given point.
        :param point: point which the gradient should be calculated to
        :return: gradient to given point
        """
        # division by 0
        if self.x == point.x:
            return None
        return float(point.y - self.y) / float(point.x - self.x)

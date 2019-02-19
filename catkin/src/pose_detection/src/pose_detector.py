#!/usr/bin/env python

from pose import Pose


class PoseDetector:
    def __init__(self, skeleton):
        self.skeleton = skeleton
        self.handGradient = None
        self.handDistance = None
        self.shoulderDistance = None
        self.handToShoulderDistance = None

    def calc_hand_gradient(self):
        if self.skeleton.keypoints[4] is not None and self.skeleton.keypoints[7] is not None:
            self.handGradient = self.skeleton.keypoints[7].gradient_to(self.skeleton.keypoints[4])
        else:
            self.handGradient = None

    def get_hand_gradient(self):
        if self.handGradient is None:
            self.calc_hand_gradient()
        return self.handGradient

    def calc_hand_distance(self):
        if self.skeleton.keypoints[4] is not None and self.skeleton.keypoints[7] is not None:
            self.handDistance = self.skeleton.keypoints[7].distance_to(self.skeleton.keypoints[4])
        else:
            self.handDistance = None

    def get_hand_distance(self):
        if self.handDistance is None:
            self.calc_hand_distance()
        return self.handDistance

    def calc_shoulder_distance(self):
        if self.skeleton.keypoints[5] is not None and self.skeleton.keypoints[2] is not None:
            self.shoulderDistance = self.skeleton.keypoints[5].distance_to(self.skeleton.keypoints[2])
        else:
            self.shoulderDistance = None

    def get_shoulder_distance(self):
        if self.shoulderDistance is None:
            self.calc_shoulder_distance()
        return self.shoulderDistance

    def calc_hand_to_shoulder_distance(self):
        if self.skeleton.keypoints[5] is not None and self.skeleton.keypoints[2] is not None and \
                self.skeleton.keypoints[7] is not None and self.skeleton.keypoints[4] is not None:
            right_distance = self.skeleton.keypoints[2].y - self.skeleton.keypoints[4].y
            left_distance = self.skeleton.keypoints[5].y - self.skeleton.keypoints[7].y
            self.handToShoulderDistance = (right_distance + left_distance) / 2
        else:
            self.handToShoulderDistance = None

    def get_hand_to_shoulder_distance(self):
        if self.handToShoulderDistance is None:
            self.calc_hand_to_shoulder_distance()
        return self.handToShoulderDistance

    def detect_pose(self):
        # Forward: both hands at shoulder height, close distance
        #  => low hand gradient, hands distance is close to shoulder distance, hands height is close to shoulders height
        if -0.3 <= self.get_hand_gradient() <= 0.3 \
                and self.get_shoulder_distance() - 200 <= self.get_hand_distance() <= self.get_shoulder_distance() + 200 \
                and -100 <= self.get_hand_to_shoulder_distance() <= 100:
            return Pose.FORWARD
        # Throttle up: both hands high, close distance
        #  => hands height is higher than shoulder height, low hand gradient, low hands distance
        elif self.get_hand_to_shoulder_distance() <= -100 \
                and -0.2 <= self.get_hand_gradient() <= 0.2 \
                and self.get_shoulder_distance() - 50 <= self.get_hand_distance() <= self.get_shoulder_distance() + 250:
            return Pose.THROTTLE_UP
        # Throttle down: both hands low, close distance
        #  => hands height is lower than shoulder height, low hand gradient, low hands distance
        elif self.get_hand_to_shoulder_distance() > 0 \
                and -0.2 <= self.get_hand_gradient() <= 0.2 \
                and self.get_shoulder_distance() - 50 <= self.get_hand_distance() <= self.get_shoulder_distance() + 250:
            return Pose.THROTTLE_DOWN
        # Right turn: left hand up, right hand down
        #  => high hand gradient
        elif self.get_hand_gradient() > 0.7 \
                and self.get_hand_distance() > 300 \
                and -200 <= self.get_hand_to_shoulder_distance() <= 200:
            return Pose.TURN_RIGHT
        # Left turn: left hand down, right hand up
        #  => high negative hand gradient
        elif self.get_hand_gradient() < -0.7 \
                and self.get_hand_distance() > 300 \
                and -200 <= self.get_hand_to_shoulder_distance() <= 200:
            return Pose.TURN_LEFT
        # Default:
        # => hold position
        else:
            return Pose.HOLD

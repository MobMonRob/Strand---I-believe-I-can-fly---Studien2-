#!/usr/bin/env python

import rospy
from instruction import Instruction


class AirsimInstructionBuilder:
    """
    Used to build instructions (throttle, roll, pitch, yaw) for AirSim.
    """

    def __init__(self):
        self.throttle = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def add_throttle(self, throttle):
        if throttle is not None:
            self.throttle += throttle

    def add_roll(self, roll):
        if roll is not None:
            self.roll += roll

    def add_pitch(self, pitch):
        if pitch is not None:
            self.pitch += pitch

    def add_yaw(self, yaw):
        if yaw is not None:
            self.yaw += yaw

    def reset_instructions(self):
        self.throttle = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def get_instructions(self):
        return {
            Instruction.THROTTLE: self.throttle,
            Instruction.ROLL: self.roll,
            Instruction.PITCH: self.pitch,
            Instruction.YAW: self.yaw
        }

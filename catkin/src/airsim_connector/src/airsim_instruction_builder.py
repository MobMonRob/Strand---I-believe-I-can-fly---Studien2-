#!/usr/bin/env python

from instruction import Instruction


class AirsimInstructionBuilder:
    """
    Used to build instructions for AirSim. Instructions are saved as their four basic components (roll, pitch, yaw and
    throttle) and can be build by adding concrete numbers to the needed axes.
    """

    def __init__(self):
        self.throttle = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def add_throttle(self, throttle):
        """
        Adds throttle to the current instruction set. Unit is measured as percentage (1.0 = 100%, 0.0 = 0%). Height
        keeping throttle depends on the other axes as pitch or roll, but 50% should be fine for just keeping the current
        height.
        :param throttle: percentage point (not percentage!) to be added to throttle
        """
        if throttle is not None:
            self.throttle += throttle

    def add_roll(self, roll):
        """
        Adds roll to the current instruction set.
        :param roll: value to be added to roll axis (roll < 0: left, roll > 0: right, roll = 0: neutral)
        """
        if roll is not None:
            self.roll += roll

    def add_pitch(self, pitch):
        """
        Adds pitch to the current instruction set.
        :param pitch: value to be added to pitch axis (pitch < 0: nose down [forward], pitch > 0: nose up [backward],
            pitch = 0: neutral)
        """
        if pitch is not None:
            self.pitch += pitch

    def add_yaw(self, yaw):
        """
        Adds yaw to the current instruction set.
        :param yaw: value to be added to yaw axis (yaw < 0: left, yaw > 0: right, yaw = 0: neutral)
        """
        if yaw is not None:
            self.yaw += yaw

    def reset_instructions(self):
        """
        Resets current instruction set to default values (all axes to 0).
        """
        self.throttle = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def get_instructions(self):
        """
        Returns current instruction set as dictionary.
        :return: current instruction set as dictionary
        """
        return {
            Instruction.THROTTLE: self.throttle,
            Instruction.ROLL: self.roll,
            Instruction.PITCH: self.pitch,
            Instruction.YAW: self.yaw
        }

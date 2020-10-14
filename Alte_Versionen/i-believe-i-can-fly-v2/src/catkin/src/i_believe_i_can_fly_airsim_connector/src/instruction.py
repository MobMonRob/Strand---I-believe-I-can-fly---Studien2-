#!/usr/bin/env python


class Instruction:
    """
    This class is used as an enum (Python does not support enums officially) for all possible flight instructions.
    """
    THROTTLE = 'THROTTLE'
    ROLL = 'ROLL'
    PITCH = 'PITCH'
    YAW = 'YAW'

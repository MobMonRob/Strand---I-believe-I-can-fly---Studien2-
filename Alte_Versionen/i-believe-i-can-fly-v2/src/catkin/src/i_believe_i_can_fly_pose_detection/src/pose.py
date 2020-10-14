#!/usr/bin/env python


class Pose:
    """
    This class is used as an enum (Python does not support enums officially) for all possible flight instructions.
    """
    HOLD = 'HOLD'
    TURN_RIGHT = 'TURN_RIGHT'
    TURN_LEFT = 'TURN_LEFT'
    THROTTLE_UP = 'THROTTLE_UP'
    THROTTLE_DOWN = 'THROTTLE_DOWN'
    FORWARD = 'FORWARD'

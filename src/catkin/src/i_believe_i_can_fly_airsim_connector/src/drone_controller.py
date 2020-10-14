#!/usr/bin/env python

import airsim
import time
import rospy
from instruction import Instruction


class DroneController:
    """
    Handles instructions for the simulated drone in AirSim.
    Basic functionality: Instructions are saved as a callback with their corresponding parameters and are called over
    time asynchronously based on a timer. If no further instructions are saved, the default instruction 'hold' should be
    called to prevent unintended flight maneuvers.
    """

    def __init__(self):
        try:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            self.update_rate = 1
            self.flying = False
            self.next_action = None
        except:
            rospy.logerr('Could not connect to running AirSim session! Is an AirSim environment really running?')
            rospy.signal_shutdown('AirSim is not running!')

    def takeoff(self, duration = 3):
        """
        Performs takeoff maneuver and updates flying status afterwards.
        """
        self.client.enableApiControl(True)
        self.client.takeoffAsync(duration).join()
        self.flying = True

    def land(self, duration = 3):
        """
        Performs landing maneuver and updates flying status afterwards.
        """
        self.client.landAsync(duration).join()
        self.client.enableApiControl(False)
        self.flying = False

    def hold(self):
        """
        Performs position holding maneuver. This maneuver should be the default position as the drone will stay at it's
        current position and does not move further on in case of missing or unclear instructions.
        """
        if self.flying:
            self.set_next_action(self.client.hoverAsync)

    def by_instruction(self, instruction):
        """
        Performs the given flight instruction in case the drone is already flying, otherwise a takeoff.
        :param instruction: dictionary containing pitch, roll, yaw and throttle values
        """
        if not self.flying and instruction[Instruction.THROTTLE] > 0:
            self.takeoff()
        elif self.flying:
            self.set_next_action(self.client.moveByAngleThrottleAsync,
                                 instruction[Instruction.PITCH],
                                 instruction[Instruction.ROLL],
                                 instruction[Instruction.THROTTLE],
                                 instruction[Instruction.YAW],
                                 self.update_rate)

    def set_next_action(self, function, *args):
        """
        Sets the given function and it's parameters as the next action which needs to be performed asynchronously.
        :param function: drone function which needs to be performed, usually 'client.moveByAngleThrottleAsync'
        :param args: arguments of the given function, usually pitch, roll, throttle and yaw
        """
        self.next_action = [function, args]

    def reset_next_action(self):
        """
        Resets the saved function which should be performed asynchronously.
        """
        self.next_action = None

    def perform_action(self):
        """
        Updates the drones instructions to the last saved flight maneuver and sleeps for the configured time.
        """
        if self.next_action is not None:
            if not self.client.isApiControlEnabled():
                self.client.enableApiControl(True)
            try:
                if self.next_action[1] is not None:
                    self.next_action[0](*self.next_action[1])
                else:
                    self.next_action[0]()
            except:
                rospy.logerr('Could not perform specified action!')
        else:
            rospy.logwarn('No upcoming action specified! Waiting for new user input.')
        time.sleep(self.update_rate)

    def print_message(self, message, severity = 0):
        """
        Prints message to the AirSim screen so the user can see it.
        :param message: message to be shown
        :param severity: severity of the message. 0 (default, info), 1 (success), 2 (error), 3 (unknown)
        """
        self.client.simPrintLogMessage(message, severity = severity)

    def reset_position(self):
        """
        Resets position of the drone and disables API control.
        """
        self.client.reset()
        self.client.enableApiControl(False)

    def release_control(self):
        """
        Releases API control of drone.
        """
        self.client.enableApiControl(False)

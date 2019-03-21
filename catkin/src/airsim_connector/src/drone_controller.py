#!/usr/bin/env python

import airsim
import time
import rospy
from instruction import Instruction


class DroneController:
    def __init__(self):
        try:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            self.update_rate = 1
            self.flying = False
            self.next_action = None
        except:
            rospy.logerr('Could not connect to running AirSim session! Is AirSim really running?')
            rospy.signal_shutdown('AirSim not running!')

    def takeoff(self):
        self.client.enableApiControl(True)
        self.client.takeoffAsync(5).join()
        self.flying = True

    def land(self):
        self.client.landAsync(5).join()
        self.client.enableApiControl(False)
        self.flying = False

    def hold(self):
        if self.flying:
            self.set_next_action(self.client.hoverAsync)

    def by_instruction(self, instruction):
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
        self.next_action = [function, args]

    def reset_next_action(self):
        self.next_action = None

    def perform_action(self):
        if self.next_action is not None:
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
        self.client.simPrintLogMessage(message, severity = severity)

#!/usr/bin/env python

import rospy
from airsim_instruction_builder import AirsimInstructionBuilder
from pose_detection.msg import Instructions as InstructionsMsg


class CommunicationManager:
    """
    Handles ROS communication with other modules and receives flight instructions.
    """

    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.instruction_builder = AirsimInstructionBuilder()

    def start_listening(self):
        rospy.Subscriber('pose_detection', InstructionsMsg, self.receive_instructions_message)

    def receive_instructions_message(self, instructions_msg):
        self.instruction_builder.reset_instructions()

        if instructions_msg.instructions is None or len(instructions_msg.instructions) == 0:
            rospy.logwarn('Unknown command!')

        for instruction in instructions_msg.instructions:
            # TODO: Use instruction builder to build proper commands. Needs probably some refactoring of the drone
            #  controller.
            # TODO: Don't use hard coded strings, reuse Pose class from pose_detection node instead.
            if instruction.instruction == 'HOLD':
                self.drone.hold()
            elif instruction.instruction == 'THROTTLE_UP':
                self.drone.throttle_up()
            elif instruction.instruction == 'THROTTLE_DOWN':
                self.drone.throttle_down()

        """
        if message.command == 'HOLD':
            self.drone.hold()
        elif message.command == 'FORWARD':
            self.drone.forward()
        elif message.command == 'TURN_LEFT':
            self.drone.left()
        elif message.command == 'TURN_RIGHT':
            self.drone.right()
        elif message.command == 'THROTTLE_UP':
            self.drone.throttle_up()
        elif message.command == 'THROTTLE_DOWN':
            self.drone.throttle_down()
        else:
            rospy.logwarn('Unknown command!')
        """

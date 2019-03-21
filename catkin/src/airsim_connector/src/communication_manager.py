#!/usr/bin/env python

import airsim
import rospy
from airsim_instruction_builder import AirsimInstructionBuilder
from instruction import Instruction
from pose_detection.msg import Calibration as CalibrationMsg
from pose_detection.msg import Instructions as InstructionsMsg


class CommunicationManager:
    """
    Handles ROS communication with other modules and receives flight instructions.
    """

    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.instruction_builder = AirsimInstructionBuilder()

    def start_listening(self):
        rospy.Subscriber('flight_instructions', InstructionsMsg, self.receive_instructions_message)
        rospy.Subscriber('calibration_status', CalibrationMsg, self.receive_calibration_status)

    def receive_instructions_message(self, instructions_msg):
        self.instruction_builder.reset_instructions()

        if instructions_msg.instructions is None or len(instructions_msg.instructions) == 0:
            rospy.logwarn('Unknown command!')

        for instruction in instructions_msg.instructions:

            # NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin

            if instruction.instruction == 'HOLD':
                self.drone.hold()
                return

            if instruction.instruction == 'THROTTLE_UP':
                scaled_throttle = (((instruction.intensity - 0.0) * (1.0 - 0.5)) / (100.0 - 0.0)) + 0.5
                self.instruction_builder.add_throttle(scaled_throttle)
            elif instruction.instruction == 'THROTTLE_DOWN':
                scaled_throttle = (((instruction.intensity - 0.0) * (0.5 - 0.0)) / (100.0 - 0.0)) + 0.0
                self.instruction_builder.add_throttle(scaled_throttle)

            if instruction.instruction == 'FORWARD':
                scaled_pitch = -1.0 * ((((instruction.intensity - 0.0) * (0.7 - 0.1)) / (100.0 - 0.0)) + 0.1)
                self.instruction_builder.add_pitch(scaled_pitch)
                self.instruction_builder.add_throttle(0.7)

            if instruction.instruction == 'TURN_LEFT':
                scaled_yaw = -1 * ((((instruction.intensity - 0.0) * (1.0 - 0.3)) / (100.0 - 0.0)) + 0.3)
                scaled_roll = -1 * ((((instruction.intensity - 0.0) * (0.6 - 0.0)) / (100.0 - 0.0)) + 0.0)
                self.instruction_builder.add_yaw(scaled_yaw)
                self.instruction_builder.add_roll(scaled_roll)
                self.instruction_builder.add_throttle(0.6)
            elif instruction.instruction == 'TURN_RIGHT':
                scaled_yaw = (((instruction.intensity - 0.0) * (1.0 - 0.3)) / (100.0 - 0.0)) + 0.3
                scaled_roll = (((instruction.intensity - 0.0) * (0.6 - 0.0)) / (100.0 - 0.0)) + 0.0
                self.instruction_builder.add_yaw(scaled_yaw)
                self.instruction_builder.add_roll(scaled_roll)
                self.instruction_builder.add_throttle(0.6)

        self.drone.by_instruction(self.instruction_builder.get_instructions())

    def receive_calibration_status(self, calibration_msg):
        if calibration_msg.status == 'STARTED_2D':
            self.drone.print_message('2D-Calibration process started!')
            self.drone.print_message('Please stand in the middle of the camera image and hold the default ' +
                                     'position for 3 seconds. Stretch both arms to the side with a 90 degree angle.')
        elif calibration_msg.status == '2D_RESET_TIMER':
            self.drone.print_message('2D-Calibration restarted! Please hold the default position for 3 seconds. ' +
                                     'Stretch both arms to the side with a 90 degree angle.', 2)
        elif calibration_msg.status == '2D_MISSING_POINTS':
            self.drone.print_message('Error during 2D-Calibration: Not all important points can be seen by the ' +
                                     'camera! Please make sure that your arms, shoulders and hip are visible for the ' +
                                     'camera.', 2)
        elif calibration_msg.status == '2D_ARMS_NOT_ON_SHOULDER_HEIGHT':
            self.drone.print_message('Error during 2D-Calibration: Your arms are not on the same height as your ' +
                                     'shoulders! Please stretch both arms to the side with a 90 degree angle!', 2)
        elif calibration_msg.status == '2D_ARMS_NOT_STRETCHED':
            self.drone.print_message('Error during 2D-Calibration: Your arms are not fully stretched out! ' +
                                     'Please stretch both arms to the side with a 90 degree angle!', 2)
        elif calibration_msg.status == 'FINISHED_2D':
            self.drone.print_message('2D-Calibration finished successfully!', 1)

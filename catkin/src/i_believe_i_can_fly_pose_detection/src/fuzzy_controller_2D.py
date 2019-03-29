#!/usr/bin/env python

import numpy as np
import rospy
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from pose import Pose


class FuzzyController2D:
    def __init__(self):
        # Calibration
        self.threshold = 10  # threshold required to trigger any flight instruction
        self.negative_threshold = -1.0 * self.threshold  # threshold for negative values
        self.calibrated_avg_arm_length = 1  # default value of 1 to avoid division by zero
        self.calibrated_shoulder_distance = 1  # default value of 1 to avoid division by zero

        # Input Variables
        # no negative distances => starting at 0
        self.hand_distance = ctrl.Antecedent(np.arange(0, 101, 1), 'hand_distance')
        # negative = hands are belows shoulders, positive = hands are above shoulders
        self.hand_to_shoulder_distance = ctrl.Antecedent(np.arange(-100, 101, 1), 'hand_to_shoulder_distance')
        # negative = left hand lower, positive = right hand lower
        self.hand_gradient = ctrl.Antecedent(np.arange(-100, 101, 1), 'hand_gradient')

        # Output Variables
        # no backwards flying => starting at 0
        self.forward = ctrl.Consequent(np.arange(0, 101, 1), 'forward')
        # negative = left turn, positive = right turn
        self.turn = ctrl.Consequent(np.arange(-100, 101, 1), 'turn')
        # negative = throttle down, positive = throttle up
        self.throttle = ctrl.Consequent(np.arange(-100, 101, 1), 'throttle')

        self.specify_input_variable_memberships()
        self.specify_output_variable_memberships()

        # Create Fuzzy Controller including rules
        self.flight_control = ctrl.ControlSystem()
        self.specify_fuzzy_rules()
        self.flight_simulation = ctrl.ControlSystemSimulation(self.flight_control)

    def specify_input_variable_memberships(self):
        """
        Specifies all input variable memberships for the Fuzzy Controller.
        """
        self.hand_distance['none'] = fuzz.trimf(self.hand_distance.universe, [0, 0, 25])
        self.hand_distance['low'] = fuzz.trimf(self.hand_distance.universe, [0, 25, 50])
        self.hand_distance['medium'] = fuzz.trimf(self.hand_distance.universe, [25, 50, 75])
        self.hand_distance['high'] = fuzz.trimf(self.hand_distance.universe, [50, 75, 100])
        self.hand_distance['vhigh'] = fuzz.trimf(self.hand_distance.universe, [75, 100, 100])

        self.hand_to_shoulder_distance['vnegative'] = fuzz.trimf(self.hand_to_shoulder_distance.universe,
                                                                 [-100, -100, -50])
        self.hand_to_shoulder_distance['negative'] = fuzz.trimf(self.hand_to_shoulder_distance.universe, [-100, -50, 0])
        self.hand_to_shoulder_distance['neutral'] = fuzz.trimf(self.hand_to_shoulder_distance.universe, [-50, 0, 50])
        self.hand_to_shoulder_distance['positive'] = fuzz.trimf(self.hand_to_shoulder_distance.universe, [0, 50, 100])
        self.hand_to_shoulder_distance['vpositive'] = fuzz.trimf(self.hand_to_shoulder_distance.universe,
                                                                 [50, 100, 100])

        self.hand_gradient['vnegative'] = fuzz.trimf(self.hand_gradient.universe, [-100, -100, -50])
        self.hand_gradient['negative'] = fuzz.trimf(self.hand_gradient.universe, [-100, -50, 0])
        self.hand_gradient['neutral'] = fuzz.trimf(self.hand_gradient.universe, [-50, 0, 50])
        self.hand_gradient['positive'] = fuzz.trimf(self.hand_gradient.universe, [0, 50, 100])
        self.hand_gradient['vpositive'] = fuzz.trimf(self.hand_gradient.universe, [50, 100, 100])

    def specify_output_variable_memberships(self):
        """
        Specifies all output variable memberships for the Fuzzy Controller.
        """
        self.forward['none'] = fuzz.trapmf(self.forward.universe, [0, 0, 20, 30])
        self.forward['slow'] = fuzz.trapmf(self.forward.universe, [20, 40, 60, 70])
        self.forward['medium'] = fuzz.trapmf(self.forward.universe, [60, 80, 90, 100])
        self.forward['fast'] = fuzz.trapmf(self.forward.universe, [80, 95, 100, 100])

        self.turn['fast_left'] = fuzz.trimf(self.turn.universe, [-100, -75, -50])
        self.turn['left'] = fuzz.trimf(self.turn.universe, [-50, -25, 0])
        self.turn['none'] = fuzz.trimf(self.turn.universe, [-25, 0, 25])
        self.turn['right'] = fuzz.trimf(self.turn.universe, [0, 25, 50])
        self.turn['fast_right'] = fuzz.trimf(self.turn.universe, [50, 75, 100])

        self.throttle['fast_down'] = fuzz.trapmf(self.throttle.universe, [-100, -100, -75, -50])
        self.throttle['down'] = fuzz.trapmf(self.throttle.universe, [-75, -45, -25, 0])
        self.throttle['medium'] = fuzz.trapmf(self.throttle.universe, [-35, -10, 10, 35])
        self.throttle['up'] = fuzz.trapmf(self.throttle.universe, [0, 25, 45, 75])
        self.throttle['fast_up'] = fuzz.trapmf(self.throttle.universe, [50, 75, 100, 100])

    def specify_fuzzy_rules(self):
        """
        Adds all rules to the Fuzzy Controller.
        """
        # Forward Flight
        self.flight_control.addrule(ctrl.Rule(self.hand_distance['none']
                                              & (self.hand_to_shoulder_distance['negative'] |
                                                 self.hand_to_shoulder_distance['neutral'] |
                                                 self.hand_to_shoulder_distance['positive']),
                                              consequent = self.forward['fast'],
                                              label = 'fast_forward_flight'))
        self.flight_control.addrule(ctrl.Rule(self.hand_distance['low']
                                              & (self.hand_to_shoulder_distance['negative'] |
                                                 self.hand_to_shoulder_distance['neutral'] |
                                                 self.hand_to_shoulder_distance['positive']),
                                              consequent = self.forward['medium'],
                                              label = 'medium_forward_flight'))
        self.flight_control.addrule(ctrl.Rule(self.hand_distance['medium']
                                              & (self.hand_to_shoulder_distance['negative'] |
                                                 self.hand_to_shoulder_distance['neutral'] |
                                                 self.hand_to_shoulder_distance['positive']),
                                              consequent = self.forward['slow'],
                                              label = 'slow_forward_flight'))
        self.flight_control.addrule(ctrl.Rule(self.hand_distance['high'],
                                              consequent = self.forward['none'],
                                              label = 'no_forward_flight_1'))
        self.flight_control.addrule(ctrl.Rule(self.hand_distance['vhigh'],
                                              consequent = self.forward['none'],
                                              label = 'no_forward_flight_2'))
        # Throttle Management
        self.flight_control.addrule(
            ctrl.Rule((self.hand_distance['none'] | self.hand_distance['low'] | self.hand_distance['medium'])
                      & self.hand_to_shoulder_distance['vnegative'],
                      consequent = self.throttle['fast_down'],
                      label = 'fast_throttle_down'))
        self.flight_control.addrule(
            ctrl.Rule((self.hand_distance['none'] | self.hand_distance['low'] | self.hand_distance['medium'])
                      & self.hand_to_shoulder_distance['negative'],
                      consequent = self.throttle['down'],
                      label = 'throttle_down'))
        self.flight_control.addrule(ctrl.Rule(self.hand_to_shoulder_distance['neutral'],
                                              consequent = self.throttle['medium'],
                                              label = 'neutral_throttle'))
        self.flight_control.addrule(
            ctrl.Rule((self.hand_distance['none'] | self.hand_distance['low'] | self.hand_distance['medium'])
                      & self.hand_to_shoulder_distance['positive'],
                      consequent = self.throttle['up'],
                      label = 'throttle_up'))
        self.flight_control.addrule(
            ctrl.Rule((self.hand_distance['none'] | self.hand_distance['low'] | self.hand_distance['medium'])
                      & self.hand_to_shoulder_distance['vpositive'],
                      consequent = self.throttle['fast_up'],
                      label = 'fast_throttle_up'))
        # Turns
        self.flight_control.addrule(ctrl.Rule(self.hand_gradient['vnegative']
                                              & (self.hand_distance['high'] | self.hand_distance['vhigh'])
                                              & (self.hand_to_shoulder_distance['negative'] |
                                                 self.hand_to_shoulder_distance['neutral'] |
                                                 self.hand_to_shoulder_distance['positive']),
                                              consequent = self.turn['fast_left'],
                                              label = 'fast_left_turn'))
        self.flight_control.addrule(ctrl.Rule(self.hand_gradient['negative']
                                              & (self.hand_distance['high'] | self.hand_distance['vhigh'])
                                              & (self.hand_to_shoulder_distance['negative'] |
                                                 self.hand_to_shoulder_distance['neutral'] |
                                                 self.hand_to_shoulder_distance['positive']),
                                              consequent = self.turn['left'],
                                              label = 'left_turn'))
        self.flight_control.addrule(ctrl.Rule(self.hand_gradient['neutral'],
                                              consequent = self.turn['none'],
                                              label = 'no_turn'))
        self.flight_control.addrule(ctrl.Rule(self.hand_gradient['positive']
                                              & (self.hand_distance['high'] | self.hand_distance['vhigh'])
                                              & (self.hand_to_shoulder_distance['negative'] |
                                                 self.hand_to_shoulder_distance['neutral'] |
                                                 self.hand_to_shoulder_distance['positive']),
                                              consequent = self.turn['right'],
                                              label = 'right_turn'))
        self.flight_control.addrule(ctrl.Rule(self.hand_gradient['vpositive']
                                              & (self.hand_distance['high'] | self.hand_distance['vhigh'])
                                              & (self.hand_to_shoulder_distance['negative'] |
                                                 self.hand_to_shoulder_distance['neutral'] |
                                                 self.hand_to_shoulder_distance['positive']),
                                              consequent = self.turn['fast_right'],
                                              label = 'fast_right_turn'))

    def calibrate(self, avg_arm_length, shoulder_distance):
        """
        Calibrates the Fuzzy Controller for a specific arm length and shoulder distance. These values are used to
        identify flight instructions from relative values. This calibration should take place before skeletons are
        processed further on.
        :param avg_arm_length: average arm length of the current user
        :param shoulder_distance: shoulder distance of the current user
        :return: True if calibration was successful, otherwise False
        """
        # avoid division by zero
        if avg_arm_length == 0 or shoulder_distance == 0:
            return False
        self.calibrated_avg_arm_length = avg_arm_length
        self.calibrated_shoulder_distance = shoulder_distance
        rospy.logdebug(
            '2D Fuzzy Controller calibrated successfully with average arm length of %f pixels and shoulder distance of %f pixels!',
            avg_arm_length, shoulder_distance)
        return True

    def get_hand_distance_percentage(self, skeleton):
        """
        Calculates the relative distance between both hands in relation to the calibrated shoulder distance.
        :param skeleton: skeleton to calculate the value for
        :return: 100 (%) equals stretched out arms, 0 (%) equals hands hold together
        """
        return 100.0 * (skeleton.get_hand_distance() / (
                2 * self.calibrated_avg_arm_length + self.calibrated_shoulder_distance))

    def get_hand_to_shoulder_distance_percentage(self, skeleton):
        """
        Calculates the relative average height distance between the hands and both shoulders in relation to the
        calibrated arm length.
        :param skeleton: skeleton to calculate the value for
        :return: -100 (%) equals both arms in the air, 100 (%) equals both arms hanging down
        """
        return 100.0 * (skeleton.get_hand_to_shoulder_distance() / float(self.calibrated_avg_arm_length))

    def get_hand_gradient_percentage(self, skeleton):
        """
        Calculates the gradient from hand to hand of the given skeleton.
        :param skeleton: skeleton to calculate the value for
        :return: 100 (%) equals 180 degree "figure" => left arm down and right arm up in the air, -100 (%) is the
                    other way around
        """
        return (skeleton.get_hand_gradient() * 100.0) / 2.0

    def detect_pose(self, skeleton):
        """
        Identifies pose for given skeleton and calculates flight instructions depending on this detected pose.
        :param skeleton: skeleton to calculate the instructions from
        :return: dictionary containing all flight instructions
        """
        self.flight_simulation.input['hand_distance'] = self.get_hand_distance_percentage(skeleton)
        self.flight_simulation.input['hand_to_shoulder_distance'] = self.get_hand_to_shoulder_distance_percentage(
            skeleton)
        self.flight_simulation.input['hand_gradient'] = self.get_hand_gradient_percentage(skeleton)
        try:
            rospy.logdebug(
                'Calculating the flight instructions with the following inputs:\n  - hand distance: %f%%\n  - hand to shoulder distance: %f%%\n  - hand gradient: %f degree',
                self.get_hand_distance_percentage(skeleton), self.get_hand_to_shoulder_distance_percentage(skeleton),
                self.get_hand_gradient_percentage(skeleton))
            self.flight_simulation.compute()

            result = dict()

            # Throttle management
            if self.flight_simulation.output['throttle'] > self.threshold:
                result[Pose.THROTTLE_DOWN] = self.flight_simulation.output['throttle']
            elif self.flight_simulation.output['throttle'] < self.negative_threshold:
                result[Pose.THROTTLE_UP] = abs(self.flight_simulation.output['throttle'])

            # Forward flight
            # TODO: Lower threshold already in Fuzzy rules and not afterwards!
            if self.flight_simulation.output['forward'] > (self.threshold * 2.0):
                # TODO: Edit actual Fuzzy rules to prevent this case from happening
                # Include forward instruction only if there is no throttle up or down instruction with a high value.
                # Allows secure landing and rise process.
                if (Pose.THROTTLE_UP in result and result[Pose.THROTTLE_UP] < 40) or (
                        Pose.THROTTLE_DOWN in result and result[Pose.THROTTLE_DOWN] < 40) or (
                        Pose.THROTTLE_UP not in result and Pose.THROTTLE_DOWN not in result):
                    result[Pose.FORWARD] = self.flight_simulation.output['forward']

            # Turns
            if self.flight_simulation.output['turn'] > self.threshold:
                result[Pose.TURN_RIGHT] = self.flight_simulation.output['turn']
            elif self.flight_simulation.output['turn'] < self.negative_threshold:
                result[Pose.TURN_LEFT] = abs(self.flight_simulation.output['turn'])

            # Hold, default when no other instruction is detected
            if not result:
                result[Pose.HOLD] = 100.0

            return result
        except ValueError:
            rospy.logwarn('No pose detected!')
            return {
                Pose.HOLD: 100.0
            }

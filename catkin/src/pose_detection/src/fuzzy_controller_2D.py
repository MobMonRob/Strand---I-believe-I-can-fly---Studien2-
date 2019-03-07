#!/usr/bin/env python

import numpy as np
import rospy
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from pose import Pose


class FuzzyController2D:
    def __init__(self):
        # Calibration
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
        self.forward['none'] = fuzz.trapmf(self.forward.universe, [0, 0, 10, 25])
        self.forward['slow'] = fuzz.trapmf(self.forward.universe, [0, 20, 40, 60])
        self.forward['medium'] = fuzz.trapmf(self.forward.universe, [40, 60, 80, 100])
        self.forward['fast'] = fuzz.trapmf(self.forward.universe, [70, 90, 100, 100])

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
                                              label = 'no_forward_flight_2'))
        self.flight_control.addrule(ctrl.Rule(self.hand_distance['vhigh'],
                                              consequent = self.forward['none'],
                                              label = 'no_forward_flight_1'))
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
        # avoid division by zero
        if avg_arm_length == 0 or shoulder_distance == 0:
            return
        self.calibrated_avg_arm_length = avg_arm_length
        self.calibrated_shoulder_distance = shoulder_distance

    def get_relative_hand_to_shoulder_distance(self, avg_hand_to_shoulder_distance):
        return avg_hand_to_shoulder_distance / self.calibrated_avg_arm_length

    def detect_pose(self, skeleton):
        self.flight_simulation.input['hand_distance'] = skeleton.get_hand_distance() / (
                2 * self.calibrated_avg_arm_length + self.calibrated_shoulder_distance)
        self.flight_simulation.input['hand_to_shoulder_distance'] = self.get_relative_hand_to_shoulder_distance(
            skeleton.get_hand_to_shoulder_distance())
        # TODO: Transform gradient to percentage
        self.flight_simulation.input['hand_gradient'] = skeleton.get_hand_gradient()
        try:
            self.flight_simulation.compute()
            # TODO: Add correct return value
            return Pose.FORWARD, 1
        except ValueError:
            rospy.logwarn('No pose detected!')
            return Pose.HOLD, 1

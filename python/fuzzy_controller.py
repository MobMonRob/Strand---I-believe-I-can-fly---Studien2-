import matplotlib.pyplot as plt
import numpy as np
import skfuzzy as fuzz
import sys
from skfuzzy import control as ctrl

# Input Variables
hand_distance = ctrl.Antecedent(np.arange(0, 101, 1), 'hand_distance')  # no negative distances => starting at 0
hand_to_shoulder_distance = ctrl.Antecedent(np.arange(-100, 101, 1), 'hand_to_shoulder_distance') # negative = hands are belows shoulders, positive = hands are above shoulders
hand_gradient = ctrl.Antecedent(np.arange(-100, 101, 1), 'hand_gradient')  # negative = left hand lower, positive = right hand lower

# Output Variables
forward = ctrl.Consequent(np.arange(0, 101, 1), 'forward')  # no backwards flying => starting at 0
turn = ctrl.Consequent(np.arange(-100, 101, 1), 'turn')  # negative = left turn, positive = right turn
throttle = ctrl.Consequent(np.arange(-100, 101, 1), 'throttle')  # negative = throttle down, positive = throttle up

# Input Variable Memberships
hand_distance['none'] = fuzz.trimf(hand_distance.universe, [0, 0, 25])
hand_distance['low'] = fuzz.trimf(hand_distance.universe, [0, 25, 50])
hand_distance['medium'] = fuzz.trimf(hand_distance.universe, [25, 50, 75])
hand_distance['high'] = fuzz.trimf(hand_distance.universe, [50, 75, 100])
hand_distance['vhigh'] = fuzz.trimf(hand_distance.universe, [75, 100, 100])

hand_to_shoulder_distance['vnegative'] = fuzz.trimf(hand_to_shoulder_distance.universe, [-100, -100, -50])
hand_to_shoulder_distance['negative'] = fuzz.trimf(hand_to_shoulder_distance.universe, [-100, -50, 0])
hand_to_shoulder_distance['neutral'] = fuzz.trimf(hand_to_shoulder_distance.universe, [-50, 0, 50])
hand_to_shoulder_distance['positive'] = fuzz.trimf(hand_to_shoulder_distance.universe, [0, 50, 100])
hand_to_shoulder_distance['vpositive'] = fuzz.trimf(hand_to_shoulder_distance.universe, [50, 100, 100])

hand_gradient['vnegative'] = fuzz.trimf(hand_gradient.universe, [-100, -100, -50])
hand_gradient['negative'] = fuzz.trimf(hand_gradient.universe, [-100, -50, 0])
hand_gradient['neutral'] = fuzz.trimf(hand_gradient.universe, [-50, 0, 50])
hand_gradient['positive'] = fuzz.trimf(hand_gradient.universe, [0, 50, 100])
hand_gradient['vpositive'] = fuzz.trimf(hand_gradient.universe, [50, 100, 100])

# Output Variable Memberships
forward['none'] = fuzz.trapmf(forward.universe, [0, 0, 10, 25])
forward['slow'] = fuzz.trapmf(forward.universe, [0, 20, 40, 60])
forward['medium'] = fuzz.trapmf(forward.universe, [40, 60, 80, 100])
forward['fast'] = fuzz.trapmf(forward.universe, [70, 90, 100, 100])

turn['fast_left'] = fuzz.trimf(turn.universe, [-100, -75, -50])
turn['left'] = fuzz.trimf(turn.universe, [-50, -25, 0])
turn['none'] = fuzz.trimf(turn.universe, [-25, 0, 25])
turn['right'] = fuzz.trimf(turn.universe, [0, 25, 50])
turn['fast_right'] = fuzz.trimf(turn.universe, [50, 75, 100])

throttle['fast_down'] = fuzz.trapmf(throttle.universe, [-100, -100, -75, -50])
throttle['down'] = fuzz.trapmf(throttle.universe, [-75, -45, -25, 0])
throttle['medium'] = fuzz.trapmf(throttle.universe, [-35, -10, 10, 35])
throttle['up'] = fuzz.trapmf(throttle.universe, [0, 25, 45, 75])
throttle['fast_up'] = fuzz.trapmf(throttle.universe, [50, 75, 100, 100])

# Create Fuzzy Controller including rules
flight_control = ctrl.ControlSystem()
# Forward Flight
flight_control.addrule(ctrl.Rule(hand_distance['none']
                       & (hand_to_shoulder_distance['negative'] | hand_to_shoulder_distance['neutral'] | hand_to_shoulder_distance['positive']),
                       consequent=forward['fast'],
                       label='fast_forward_flight'))
flight_control.addrule(ctrl.Rule(hand_distance['low']
                       & (hand_to_shoulder_distance['negative'] | hand_to_shoulder_distance['neutral'] | hand_to_shoulder_distance['positive']),
                       consequent=forward['medium'],
                       label='medium_forward_flight'))
flight_control.addrule(ctrl.Rule(hand_distance['medium']
                       & (hand_to_shoulder_distance['negative'] | hand_to_shoulder_distance['neutral'] | hand_to_shoulder_distance['positive']),
                       consequent=forward['slow'],
                       label='slow_forward_flight'))
flight_control.addrule(ctrl.Rule(hand_distance['high'],
                       consequent=forward['none'],
                       label='no_forward_flight_2'))
flight_control.addrule(ctrl.Rule(hand_distance['vhigh'],
                       consequent=forward['none'],
                       label='no_forward_flight_1'))
# Throttle Management
flight_control.addrule(ctrl.Rule((hand_distance['none'] | hand_distance['low'] | hand_distance['medium'])
                       & hand_to_shoulder_distance['vnegative'],
                       consequent=throttle['fast_down'],
                       label='fast_throttle_down'))
flight_control.addrule(ctrl.Rule((hand_distance['none'] | hand_distance['low'] | hand_distance['medium'])
                       & hand_to_shoulder_distance['negative'],
                       consequent=throttle['down'],
                       label='throttle_down'))
flight_control.addrule(ctrl.Rule(hand_to_shoulder_distance['neutral'],
                       consequent=throttle['medium'],
                       label='neutral_throttle'))
flight_control.addrule(ctrl.Rule((hand_distance['none'] | hand_distance['low'] | hand_distance['medium'])
                       & hand_to_shoulder_distance['positive'],
                       consequent=throttle['up'],
                       label='throttle_up'))
flight_control.addrule(ctrl.Rule((hand_distance['none'] | hand_distance['low'] | hand_distance['medium'])
                       & hand_to_shoulder_distance['vpositive'],
                       consequent=throttle['fast_up'],
                       label='fast_throttle_up'))
# Turns
flight_control.addrule(ctrl.Rule(hand_gradient['vnegative']
                       & (hand_distance['high'] | hand_distance['vhigh'])
                       & (hand_to_shoulder_distance['negative'] | hand_to_shoulder_distance['neutral'] | hand_to_shoulder_distance['positive']),
                       consequent=turn['fast_left'],
                       label='fast_left_turn'))
flight_control.addrule(ctrl.Rule(hand_gradient['negative']
                       & (hand_distance['high'] | hand_distance['vhigh'])
                       & (hand_to_shoulder_distance['negative'] | hand_to_shoulder_distance['neutral'] | hand_to_shoulder_distance['positive']),
                       consequent=turn['left'],
                       label='left_turn'))
flight_control.addrule(ctrl.Rule(hand_gradient['neutral'],
                       consequent=turn['none'],
                       label='no_turn'))
flight_control.addrule(ctrl.Rule(hand_gradient['positive']
                       & (hand_distance['high'] | hand_distance['vhigh'])
                       & (hand_to_shoulder_distance['negative'] | hand_to_shoulder_distance['neutral'] | hand_to_shoulder_distance['positive']),
                       consequent=turn['right'],
                       label='right_turn'))
flight_control.addrule(ctrl.Rule(hand_gradient['vpositive']
                       & (hand_distance['high'] | hand_distance['vhigh'])
                       & (hand_to_shoulder_distance['negative'] | hand_to_shoulder_distance['neutral'] | hand_to_shoulder_distance['positive']),
                       consequent=turn['fast_right'],
                       label='fast_right_turn'))

# Calculate Output
flight_simulation = ctrl.ControlSystemSimulation(flight_control)
flight_simulation.input['hand_distance'] = 80
flight_simulation.input['hand_to_shoulder_distance'] = 0
flight_simulation.input['hand_gradient'] = 75

try:
    flight_simulation.compute()
except ValueError:
    print('No pose detected!')
    sys.exit()

# Plot Output
hand_distance.view(sim=flight_simulation)
hand_to_shoulder_distance.view(sim=flight_simulation)
hand_gradient.view(sim=flight_simulation)

forward.view(sim=flight_simulation)
throttle.view(sim=flight_simulation)
turn.view(sim=flight_simulation)

plt.show()

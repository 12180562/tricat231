#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

def fuzzy(self):
    # Define input variables
    distance = ctrl.Antecedent(np.arange(0, 4, 0.1), "distance")
    angle = ctrl.Antecedent(np.arange(-70, 70, 1), "angle")
    
    # Define output variable
    target_servo = ctrl.Consequent(np.arange(-30, 30, 1), "target_servo")
    
    # Define membership functions for input variables
    ## distance
    distance["ED"] = fuzz.trapmf(distance.universe, [0, 0, 0.5, 1.5])
    distance["D"] = fuzz.trimf(distance.universe, [0.5, 1.5, 2.5])
    distance["W"] = fuzz.trimf(distance.universe, [1.5, 2.5, 3.5])
    distance["B"] = fuzz.trapmf(distance.universe, [2.5, 3.5, 4, 4])
    ## angle
    angle["NL"] = fuzz.trapmf(angle.universe, [-70, -50, -40, -30]) # Negative Large
    angle["NM"] = fuzz.trapmf(angle.universe, [-40, -30, -20, -10]) # Negative Medium
    angle["NS"] = fuzz.trimf(angle.universe, [-25, 0, 1])           # Negative Small
    angle["PS"] = fuzz.trimf(angle.universe, [0, 1, 25])            # Positive Small
    angle["PM"] = fuzz.trapmf(angle.universe, [10, 20, 30, 40])     # Positive Medium
    angle["PL"] = fuzz.trapmf(angle.universe, [30, 40, 50, 70])     # Positive Large

    # Define membership functions for output variable
    ## servo motor
    target_servo["RE"] = fuzz.trimf(target_servo.universe, [-30, -27, -18]) # Rigtht Extra
    target_servo["RL"] = fuzz.trimf(target_servo.universe, [-18, -16, -13]) # Right Large
    target_servo["RM"] = fuzz.trimf(target_servo.universe, [-18, -13, -7])  # Right Medium
    target_servo["RS"] = fuzz.trimf(target_servo.universe, [-12, -7, 0])    # Right Small
    target_servo["N"] = fuzz.trimf(target_servo.universe, [0, 0, 0])
    target_servo["LS"] = fuzz.trimf(target_servo.universe, [0, 7, 12])      # Left Small
    target_servo["LM"] = fuzz.trimf(target_servo.universe, [7, 13, 18])     # Left Medium
    target_servo["LL"] = fuzz.trimf(target_servo.universe, [13, 16, 18])    # Left Large
    target_servo["LE"] = fuzz.trimf(target_servo.universe, [18, 27, 30])    # Left Extra

    # Define rules
    rule_ED_NL = ctrl.Rule(distance["ED"] & angle["NL"], target_servo["RM"])
    rule_ED_NM = ctrl.Rule(distance["ED"] & angle["NM"], target_servo["RL"])
    rule_ED_NS = ctrl.Rule(distance["ED"] & angle["NS"], target_servo["RE"])
    rule_ED_PS = ctrl.Rule(distance["ED"] & angle["PS"], target_servo["LE"])
    rule_ED_PM = ctrl.Rule(distance["ED"] & angle["PM"], target_servo["LL"])
    rule_ED_PL = ctrl.Rule(distance["ED"] & angle["PL"], target_servo["LM"])

    rule_D_NL = ctrl.Rule(distance["D"] & angle["NL"], target_servo["RS"])
    rule_D_NM = ctrl.Rule(distance["D"] & angle["NM"], target_servo["RM"])
    rule_D_NS = ctrl.Rule(distance["D"] & angle["NS"], target_servo["RL"])
    rule_D_PS = ctrl.Rule(distance["D"] & angle["PS"], target_servo["LL"])
    rule_D_PM = ctrl.Rule(distance["D"] & angle["PM"], target_servo["LM"])
    rule_D_PL = ctrl.Rule(distance["D"] & angle["PL"], target_servo["LS"])

    rule_W_NL = ctrl.Rule(distance["W"] & angle["NL"], target_servo["N"])
    rule_W_NM = ctrl.Rule(distance["W"] & angle["NM"], target_servo["RS"])
    rule_W_NS = ctrl.Rule(distance["W"] & angle["NS"], target_servo["RM"])
    rule_W_PS = ctrl.Rule(distance["W"] & angle["PS"], target_servo["LM"])
    rule_W_PM = ctrl.Rule(distance["W"] & angle["PM"], target_servo["LS"])
    rule_W_PL = ctrl.Rule(distance["W"] & angle["PL"], target_servo["N"])

    rule_B_NL = ctrl.Rule(distance["B"] & angle["NL"], target_servo["N"])
    rule_B_NM = ctrl.Rule(distance["B"] & angle["NM"], target_servo["N"])
    rule_B_NS = ctrl.Rule(distance["B"] & angle["NS"], target_servo["RS"])
    rule_B_PS = ctrl.Rule(distance["B"] & angle["PS"], target_servo["LS"])
    rule_B_PM = ctrl.Rule(distance["B"] & angle["PM"], target_servo["N"])
    rule_B_PL = ctrl.Rule(distance["B"] & angle["PL"], target_servo["N"])

    # Base class to contain a Fuzzy Control System
    target_servo_ctrl = ctrl.ControlSystem(
        [rule_ED_NL, rule_ED_NM, rule_ED_NS, rule_ED_PL, rule_ED_PM, rule_ED_PS,
            rule_D_NL, rule_D_NM, rule_D_NS, rule_D_PL, rule_D_PM, rule_D_PS,
            rule_W_NL, rule_W_NM, rule_W_NS, rule_W_PL, rule_W_PM, rule_W_PS,
            rule_B_NL, rule_B_NM, rule_B_NS, rule_B_PL, rule_B_PM, rule_B_PS])
    
    self.target_servo_ang = ctrl.ControlSystemSimulation(target_servo_ctrl)

def fuzzy_control_avoidance(self):
    self.danger_ob = {}

    start_idx = int((math.radians(110)) / (self.angle_increment + 0.00001))
    end_idx = int((math.radians(250)) / (self.angle_increment + 0.00001))
    ranges = self.ranges[start_idx : (end_idx + 1)]

    if ranges == [] or min(ranges) == float("inf"):
        return False

    closest_distance = min(ranges)

    self.clo=closest_distance

    idx = ranges.index(closest_distance) + start_idx
    pi = math.degrees(self.angle_min + self.angle_increment * idx)

    if (0.3 <= closest_distance <= 2.8) and (-70 <= pi <= 70):
        self.target_servo_ang.input["distance"] = float(closest_distance)
        self.target_servo_ang.input["angle"] = float(pi)
        self.target_servo_ang.compute()
        self.fuzzy_servo_control = int(self.target_servo_ang.output["target_servo"])

        self.danger_ob["distance"] = closest_distance
        self.danger_ob["idx"] = idx
        self.danger_ob["pi"] = pi

        return True
    else:
        return False
    
def print_state(self):
    print(f"------------------------------------\n \
        lpp : {self.fuzzy_control_avoidance()}\n \
        distance, thruster : {self.distance_goal}, {self.u_thruster}\n \
        my xy : {self.boat_x}, {self.boat_y}\n \
        goal xy : {self.goal_x}, {self.goal_y}\n \
        psi, desire : {self.psi}, {self.psi_desire}\n \
        servo : {self.servo_pid_controller()-93}\n \
        range : {self.clo}\n \
        target rule : {self.fuzzy_servo_control}")
#!/usr/bin/env python3

class WheelConfiguration:
    """
    class holding configuration for wheel
    """
    def __init__(self):
        self.diameter = None
        self.motor_configuration = MotorConfiguration()


class MotorConfiguration:
    """
    class holding configuration for motor
    """
    def __init__(self):
        self.pole_pairs = None
        self.max_torque = None
        self.hall_configuration = MotorHallConfiguration()
        self.motion_configuration = MotorMotionConfiguration()
        self.pid_configuration = MotorPIDConfiguration()


class MotorHallConfiguration:
    """
    class holding configuration for motor hall controller
    """
    def __init__(self):
        self.hall_invert = None


class MotorMotionConfiguration:
    """
    class holding configuration for motor motion controller
    """
    def __init__(self):
        self.max_velocity = None
        self.acceleration = None
        self.ramp_enable = None
        self.target_reached_velocity = None
        self.target_reached_distance = None
        self.motor_halted_velocity = None


class MotorPIDConfiguration:
    """
    class holding configuration for motor PID controller
    """
    def __init__(self):
        self.torque_p = None
        self.torque_i = None
        self.velocity_p = None
        self.velocity_i = None
        self.position_parameter = None

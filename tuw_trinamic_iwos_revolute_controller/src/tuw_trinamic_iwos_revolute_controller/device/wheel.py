#!/usr/bin/env python3

import math

from tuw_trinamic_iwos_revolute_controller.device.motor import Motor
from tuw_trinamic_iwos_revolute_controller.device.configuration_tool import ConfigurationTool


class Wheel:
    """
    class representing a wheel controlled with Trinamic TMCM-1640
    """
    def __init__(self, usb_port, path_to_configuration):
        self._path_to_configuration = path_to_configuration
        self._usb_port = usb_port
        self._configuration = None
        self._perimeter = None
        self._motor = None

        self._load_configuration()
        self._setup_perimeter()
        self._setup_motor()

    def set_velocity(self, velocity):
        """
        set the rounded target velocity (m/s) for the wheel
        :param velocity: target velocity (m/s)
        :return:
        """
        # velocity needs to be multiplied by negative one to change direction (since wheels are mounted in reverse)
        velocity_ms = velocity * -1
        velocity_rps = velocity_ms / self._perimeter
        velocity_rpm = velocity_rps * 60
        self._motor.set_target_velocity_rpm(velocity=round(velocity_rpm))

    def _load_configuration(self):
        self._configuration = ConfigurationTool.read_configuration(path_to_configuration=self._path_to_configuration)

    def _setup_perimeter(self):
        self._perimeter = self._configuration.diameter * math.pi

    def _setup_motor(self):
        self._motor = Motor(usb_port=self._usb_port, configuration=self._configuration.motor_configuration)

#!/usr/bin/env python3

from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.modules.TMCM1640.TMCM_1640 import TMCM_1640

from tuw_trinamic_controller.exception.invalid_configuration_exception import InvalidConfigurationException


class Motor:
    """
    class representing a motor controlled with Trinamic TMCM-1640
    """
    def __init__(self, usb_port, configuration):
        if configuration is None:
            raise InvalidConfigurationException("Invalid motor configuration")

        self._module_connection = ConnectionManager(argList=usb_port).connect()
        self._module = TMCM_1640(connection=self._module_connection)
        self._motor = self._module.motor(motorID=0)
        self._configuration = configuration
        self._configure()

    def set_target_velocity_rpm(self, velocity):
        """
        set the rounded target velocity (rpm) for the motor
        :param velocity: target velocity (rpm)
        :return:
        """
        self._motor.setTargetVelocity(velocity=round(velocity))

    def _configure(self):
        self._set_pole_pairs()
        self._set_hall_inverter()
        self._set_maximum_velocity()
        self._set_maximum_acceleration()
        self._set_maximum_torque()
        self._set_ramp()
        self._set_target_reached_distance()
        self._set_target_reached_velocity()
        self._set_motor_halted_velocity()
        self._set_torque_pi()
        self._set_velocity_pi()
        self._set_position_parameter()

    def _set_pole_pairs(self):
        self._motor.setMotorPolePairs(polePairs=self._configuration.pole_pairs)

    def _set_hall_inverter(self):
        self._motor.digitalHall.setHallInvert(invert=self._configuration.hall_configuration.hall_invert)

    def _set_maximum_velocity(self):
        self._motor.linearRamp.setMaxVelocity(maxVelocity=self._configuration.motion_configuration.max_velocity)

    def _set_maximum_acceleration(self):
        self._motor.linearRamp.setAcceleration(acceleration=self._configuration.motion_configuration.acceleration)

    def _set_maximum_torque(self):
        self._motor.setMaxTorque(maxTorque=self._configuration.max_torque)

    def _set_ramp(self):
        self._motor.linearRamp.setRampEnabled(enable=self._configuration.motion_configuration.ramp_enable)

    def _set_target_reached_distance(self):
        self._motor.linearRamp.setTargetReachedDistance(
            distance=self._configuration.motion_configuration.target_reached_distance)

    def _set_target_reached_velocity(self):
        self._motor.linearRamp.setTargetReachedVelocity(
            velocity=self._configuration.motion_configuration.target_reached_velocity)

    def _set_motor_halted_velocity(self):
        self._motor.linearRamp.setMotorHaltedVelocity(
            velocity=self._configuration.motion_configuration.motor_halted_velocity)

    def _set_torque_pi(self):
        self._motor.pid.setTorquePIParameter(
            pValue=self._configuration.pid_configuration.torque_p,
            iValue=self._configuration.pid_configuration.torque_i)

    def _set_velocity_pi(self):
        self._motor.pid.setVelocityPIParameter(
            pValue=self._configuration.pid_configuration.velocity_p,
            iValue=self._configuration.pid_configuration.velocity_i)

    def _set_position_parameter(self):
        self._motor.pid.setPositionPParameter(pValue=self._configuration.pid_configuration.position_parameter)

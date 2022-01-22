#!/usr/bin/env python3

import math
import rospy

from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.modules.TMCM1640.TMCM_1640 import TMCM_1640

from tuw_trinamic_iwos_revolute_controller.exception.invalid_config_exception import InvalidConfigException


class TrinamicConnection:
    def __init__(self, usb_port, log_prefix):
        self._node_name = rospy.get_name()
        self._module_connection = ConnectionManager(argList=usb_port).connect()
        self._module = TMCM_1640(connection=self._module_connection)
        self._motor = self._module.motor(motorID=0)
        self.config = None

    def set_config(self, config):
        self.config = config
        self._set_motor_pole_pairs(motor_pole_pairs=config.motor_pole_pairs)
        self._set_digital_hall_inverter(digital_hall_invert=config.digital_hall_invert)
        self._set_max_velocity(max_velocity=config.max_velocity)
        self._set_max_torque(max_torque=config.max_torque)
        self._set_acceleration(acceleration=config.acceleration)
        self._set_ramp_enable(ramp_enable=config.ramp_enable)
        self._set_target_reached_distance(target_reached_distance=config.target_reached_distance)
        self._set_target_reached_velocity(target_reached_velocity=config.target_reached_velocity)
        self._set_motor_halted_velocity(motor_halted_velocity=config.motor_halted_velocity)
        self._set_position_p_parameter(position_p_parameter=config.position_p_parameter)
        self._set_velocity_p_parameter(velocity_p=config.velocity_p_parameter)
        self._set_velocity_i_parameter(velocity_i=config.velocity_i_parameter)
        self._set_torque_p_parameter(torque_p=config.torque_p_parameter)
        self._set_torque_i_parameter(torque_i=config.torque_i_parameter)
        return self._check_config(config=config)

    def fetch_config(self):
        self.config.motor_pole_pairs = self._get_motor_pole_pairs()
        self.config.digital_hall_invert = self._get_digital_hall_inverter()
        self.config.max_velocity = self._get_max_velocity()
        self.config.max_torque = self._get_max_torque()
        self.config.acceleration = self._get_acceleration()
        self.config.ramp_enable = self._get_ramp_enable()
        self.config.target_reached_distance = self._get_target_reached_distance()
        self.config.target_reached_velocity = self._get_target_reached_velocity()
        self.config.motor_halted_velocity = self._get_motor_halted_velocity()
        self.config.position_p_parameter = self._get_position_p_parameter()
        self.config.velocity_p_parameter = self._get_velocity_p_parameter()
        self.config.velocity_i_parameter = self._get_velocity_i_parameter()
        self.config.torque_p_parameter = self._get_torque_p_parameter()
        self.config.torque_i_parameter = self._get_torque_i_parameter()
        return self.config

    def set_target_velocity_rpm(self, target_velocity_rpm):
        self._motor.setTargetVelocity(velocity=round(target_velocity_rpm))

    def set_target_velocity(self, target_velocity):
        target_velocity_ms = target_velocity * -1
        target_velocity_rps = target_velocity_ms / (self.config.wheel_diameter * math.pi)
        target_velocity_rpm = target_velocity_rps * 60
        target_velocity_rpm = round(target_velocity_rpm)
        self._motor.set_target_velocity_rpm(velocity=target_velocity_rpm)

    def _check_config_value(self, should, actual, string):
        if should != actual:
            rospy.logerr('{node_name}: failed to configure {string}'.format(node_name=self._node_name, string=string))
            raise InvalidConfigException()

    def _check_config(self, config):
        self._check_config_value(should=config.motor_pole_pairs, actual=self._get_motor_pole_pairs(),
                                 string='motor pole pairs')
        self._check_config_value(should=config.digital_hall_invert, actual=self._get_digital_hall_inverter(),
                                 string='digital hall inverter')
        self._check_config_value(should=config.max_velocity, actual=self._get_max_velocity(),
                                 string='max velocity')
        self._check_config_value(should=config.max_torque, actual=self._get_max_torque(),
                                 string='max torque')
        self._check_config_value(should=config.acceleration, actual=self._get_acceleration(),
                                 string='acceleration')
        self._check_config_value(should=config.ramp_enable, actual=self._get_ramp_enable(),
                                 string='ramp enable')
        self._check_config_value(should=config.target_reached_distance, actual=self._get_target_reached_distance(),
                                 string='target reached distance')
        self._check_config_value(should=config.target_reached_velocity, actual=self._get_target_reached_velocity(),
                                 string='target reached velocity')
        self._check_config_value(should=config.motor_halted_velocity, actual=self._get_motor_halted_velocity(),
                                 string='motor halted velocity')
        self._check_config_value(should=config.position_p_parameter, actual=self._get_position_p_parameter(),
                                 string='position p parameter')
        self._check_config_value(should=config.velocity_p_parameter, actual=self._get_velocity_p_parameter(),
                                 string='velocity p parameter')
        self._check_config_value(should=config.velocity_i_parameter, actual=self._get_velocity_i_parameter(),
                                 string='velocity i parameter')
        self._check_config_value(should=config.torque_p_parameter, actual=self._get_torque_p_parameter(),
                                 string='torque p parameter')
        self._check_config_value(should=config.torque_i_parameter, actual=self._get_torque_i_parameter(),
                                 string='torque i parameter')
        return config

    def _set_motor_pole_pairs(self, motor_pole_pairs):
        self._motor.setMotorPolePairs(polePairs=motor_pole_pairs)

    def _get_motor_pole_pairs(self):
        return self._motor.motorPolePairs()

    def _set_digital_hall_inverter(self, digital_hall_invert):
        self._motor.digitalHall.setHallInvert(invert=digital_hall_invert)

    def _get_digital_hall_inverter(self):
        return self._motor.digialHall.hallInverter()

    def _set_max_velocity(self, max_velocity):
        self._motor.linearRamp.setMaxVelocity(maxVelocity=max_velocity)

    def _get_max_velocity(self):
        return self._motor.linearRamp.maxVelocity()

    def _set_max_torque(self, max_torque):
        self._motor.setMaxTorque(maxTorque=max_torque)

    def _get_max_torque(self):
        return self._motor.maxTorque()

    def _set_acceleration(self, acceleration):
        self._motor.linearRamp.setAcceleration(acceleration=acceleration)

    def _get_acceleration(self):
        return self._motor.linearRamp.acceleration()

    def _set_ramp_enable(self, ramp_enable):
        self._motor.linearRamp.setRampEnabled(enable=ramp_enable)

    def _get_ramp_enable(self):
        return self._motor.linearRamp.rampEnabled()

    def _set_target_reached_distance(self, target_reached_distance):
        self._motor.linearRamp.setTargetReachedDistance(distance=target_reached_distance)

    def _get_target_reached_distance(self):
        return self._motor.linearRamp.targetReachedDistance()

    def _set_target_reached_velocity(self, target_reached_velocity):
        self._motor.linearRamp.setTargetReachedVelocity(velocity=target_reached_velocity)

    def _get_target_reached_velocity(self):
        return self._motor.linearRamp.targetReachedVelocity()

    def _set_motor_halted_velocity(self, motor_halted_velocity):
        self._motor.linearRamp.setMotorHaltedVelocity(velocity=motor_halted_velocity)

    def _get_motor_halted_velocity(self):
        return self._motor.linearRamp.motorHaltedVelocity()

    def _set_position_p_parameter(self, position_p_parameter):
        self._motor.pid.setPositionPParameter(pValue=position_p_parameter)

    def _get_position_p_parameter(self):
        return self._motor.pid.positionPParameter()

    def _set_velocity_p_parameter(self, velocity_p):
        self._motor.pid.setVelocityPParameter(pValue=velocity_p)

    def _get_velocity_p_parameter(self):
        return self._motor.pid.velocityPParameter()

    def _set_velocity_i_parameter(self, velocity_i):
        self._motor.pid.setVelocityIParameter(iValue=velocity_i)

    def _get_velocity_i_parameter(self):
        return self._motor.pid.velocityIParameter()

    def _set_torque_p_parameter(self, torque_p):
        self._motor.pid.setTorquePParameter(pValue=torque_p)

    def _get_torque_p_parameter(self):
        return self._motor.pid.torquePParameter()

    def _set_torque_i_parameter(self, torque_i):
        self._motor.pid.setTorqueIParameter(iValue=torque_i)

    def _get_torque_i_parameter(self):
        return self._motor.pid.torqueIParameter()

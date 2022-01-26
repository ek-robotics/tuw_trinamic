#!/usr/bin/env python3

import threading

import rospy
from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.modules.TMCM1640.TMCM_1640 import TMCM_1640

from tuw_trinamic_controller.connection.abstract_trinamic_connection import AbstractTrinamicConnection
from tuw_trinamic_controller.exception.invalid_config_exception import InvalidConfigException


class TrinamicTMCM1640Connection(AbstractTrinamicConnection):

    def __init__(self, port, config_type):
        self._lock = threading.Lock()
        self._port = port
        self._config_type = config_type
        self._module_connection = ConnectionManager(argList='--port ' + self._port).connect()
        self._module = TMCM_1640(connection=self._module_connection)
        self._motor = self._module.motor(motorID=0)

    def set_command(self, command):
        self._lock.acquire()
        self._set_target_velocity(target_velocity=command)
        self._lock.release()

    def get_state(self):
        self._lock.acquire()
        state = {
            'name': self._port,
            'position': self._get_position(),
            'velocity': self._get_velocity(),
            'effort': self._get_torque()
        }
        self._lock.release()
        return state

    def set_config(self, config):
        self._lock.acquire()
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
        self._lock.release()

        actual_config = self.get_config()
        if config.equals(actual_config):
            return config

        if not config.equals(actual_config):
            return None

    def get_config(self):
        config = self._config_type()
        self._lock.acquire()
        config.motor_pole_pairs = self._get_motor_pole_pairs()
        config.digital_hall_invert = self._get_digital_hall_inverter()
        config.max_velocity = self._get_max_velocity()
        config.max_torque = self._get_max_torque()
        config.acceleration = self._get_acceleration()
        config.ramp_enable = self._get_ramp_enable()
        config.target_reached_distance = self._get_target_reached_distance()
        config.target_reached_velocity = self._get_target_reached_velocity()
        config.motor_halted_velocity = self._get_motor_halted_velocity()
        config.position_p_parameter = self._get_position_p_parameter()
        config.position_i_parameter = self._get_velocity_p_parameter()
        config.velocity_i_parameter = self._get_velocity_i_parameter()
        config.torque_p_parameter = self._get_torque_p_parameter()
        config.torque_i_parameter = self._get_torque_i_parameter()
        self._lock.release()
        return config

    def _get_position(self):
        return self._motor.actualPosition()

    def _get_velocity(self):
        return self._motor.actualVelocity()

    def _get_torque(self):
        return self._motor.actualTorque()

    def _set_target_velocity(self, target_velocity):
        self._motor.setTargetVelocity(velocity=target_velocity)

    def _get_target_velocity(self):
        return self._motor.targetVelocity()

    def _set_motor_pole_pairs(self, motor_pole_pairs):
        self._motor.setMotorPolePairs(polePairs=motor_pole_pairs)

    def _get_motor_pole_pairs(self):
        return self._motor.motorPolePairs()

    def _set_digital_hall_inverter(self, digital_hall_invert):
        self._motor.digitalHall.setHallInvert(invert=digital_hall_invert)

    def _get_digital_hall_inverter(self):
        return self._motor.digitalHall.hallInvert()

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

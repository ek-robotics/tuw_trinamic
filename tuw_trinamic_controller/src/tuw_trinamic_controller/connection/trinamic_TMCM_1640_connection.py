#!/usr/bin/env python3

import threading

from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.modules.TMCM1640.TMCM_1640 import TMCM_1640

from tuw_trinamic_controller.connection.abstract_trinamic_connection import AbstractTrinamicConnection


class TrinamicTMCM1640Connection(AbstractTrinamicConnection):

    def __init__(self, port, baudrate, config_type):
        self._lock = threading.Lock()
        self._port = port
        self._baudrate = baudrate
        self._config_type = config_type
        arg_list = '--port {port} --data-rate {baudrate}'.format(port=self._port, baudrate=self._baudrate)
        self._module_connection = ConnectionManager(argList=arg_list).connect()
        self._module = TMCM_1640(connection=self._module_connection)
        self._motor = self._module.motor(motorID=0)
        self._last_command = None

    def set_command(self, command):
        if self._last_command is None:
            self._last_command = command
            self._lock.acquire()
            self._set_command_position(command=command.velocity[0])
            self._set_command_velocity(command=command.velocity[0])
            self._set_command_torque(command=command.torque[0])
            self._lock.release()
            return True

        elif command.header.seq > self._last_command.header.seq:
            self._lock.acquire()
            if command.position[0] != self._last_command.position[0]:
                self._set_command_position(command=command.position[0])
            if command.velocity[0] != self._last_command.velocity[0]:
                self._set_command_velocity(command=command.velocity[0])
            if command.torque[0] != self._last_command.torque[0]:
                self._set_command_torque(command=command.torque[0])
            self._lock.release()

            self._last_command = command
            return True

        elif command.header.seq < self._last_command.header.seq:
            self._last_command = command
            return False

        elif command.header.seq == self._last_command.header.seq:
            self._last_command = command
            return False

    def _set_command_position(self, command):
        if command is not None:
            self._set_target_position(target_position=round(command))

    def _set_command_velocity(self, command):
        if command is not None:
            self._set_target_velocity(target_velocity=round(command))

    def _set_command_torque(self, command):
        # NOTE: command torque is interpreted as maximum torque
        if command is not None:
            self._set_max_torque(max_torque=round(command))

    def get_state(self):
        self._lock.acquire()
        state = {
            'position': self._get_position(),
            'velocity': self._get_velocity(),
            'torque': self._get_torque()
        }
        self._lock.release()
        return state

    def set_config(self, config, verify=True):
        self._lock.acquire()
        self._set_config(function=self._set_motor_pole_pairs, value=config.motor_pole_pairs)
        self._set_config(function=self._set_digital_hall_inverter, value=config.digital_hall_invert)
        self._set_config(function=self._set_max_velocity, value=config.max_velocity)
        self._set_config(function=self._set_max_torque, value=config.max_torque)
        self._set_config(function=self._set_acceleration, value=config.acceleration)
        self._set_config(function=self._set_ramp_enable, value=config.ramp_enable)
        self._set_config(function=self._set_target_reached_distance, value=config.target_reached_distance)
        self._set_config(function=self._set_target_reached_velocity, value=config.target_reached_velocity)
        self._set_config(function=self._set_motor_halted_velocity, value=config.motor_halted_velocity)
        self._set_config(function=self._set_position_p_parameter, value=config.position_p_parameter)
        self._set_config(function=self._set_velocity_p_parameter, value=config.velocity_p_parameter)
        self._set_config(function=self._set_velocity_i_parameter, value=config.velocity_i_parameter)
        self._set_config(function=self._set_torque_p_parameter, value=config.torque_p_parameter)
        self._set_config(function=self._set_torque_i_parameter, value=config.torque_i_parameter)
        self._lock.release()

        actual_config = self.get_config()

        if not actual_config.all_set():
            return None

        if verify:
            return actual_config if config.equals(actual_config) else None

        if not verify:
            return actual_config

    @staticmethod
    def _set_config(function, value):
        if value is not None:
            function(value)

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
        config.velocity_p_parameter = self._get_velocity_p_parameter()
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

    def _set_target_position(self, target_position):
        self._motor.setTargetPosition(position=target_position)

    def _set_target_velocity(self, target_velocity):
        self._motor.setTargetVelocity(velocity=target_velocity)

    def _set_target_torque(self, target_torque):
        self._motor.setTargetTorque(torque=target_torque)

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

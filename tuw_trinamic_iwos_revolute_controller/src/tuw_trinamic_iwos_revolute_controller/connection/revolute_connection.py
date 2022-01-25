#!/usr/bin/env python3

import math
import rospy

from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.modules.TMCM1640.TMCM_1640 import TMCM_1640
from sensor_msgs.msg import JointState

from tuw_trinamic_iwos_revolute_controller.exception.invalid_config_exception import InvalidConfigException


class RevoluteConnection:
    def __init__(self, usb_port):
        self._node_name = rospy.get_name()
        self._config = None
        self._module_connection = ConnectionManager(argList='--port ' + usb_port).connect()
        self._module = TMCM_1640(connection=self._module_connection)
        self._motor = self._module.motor(motorID=0)

    def set_target_velocity(self, target_velocity):
        target_velocity_ms = target_velocity * -1
        target_velocity_rps = target_velocity_ms / (self._config.wheel_diameter * math.pi)
        target_velocity_rpm = target_velocity_rps * 60
        target_velocity_rpm = round(target_velocity_rpm)
        self._set_target_velocity(target_velocity=target_velocity_rpm)

    def get_state(self, name):
        joint_state = JointState()
        joint_state.name.append(name)
        joint_state.position.append(self._get_position())
        joint_state.velocity.append(self._get_velocity())
        joint_state.effort.append(self._get_torque())
        return joint_state

    def set_config(self, config):
        self._config = config
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
        return self._verify_config(config=config)

    def _verify_config(self, config):
        self._verify_config_value(should=config.motor_pole_pairs,
                                  actual=self._get_motor_pole_pairs(),
                                  name='motor pole pairs')
        self._verify_config_value(should=config.digital_hall_invert,
                                  actual=self._get_digital_hall_inverter(),
                                  name='digital hall inverter')
        self._verify_config_value(should=config.max_velocity,
                                  actual=self._get_max_velocity(),
                                  name='max velocity')
        self._verify_config_value(should=config.max_torque,
                                  actual=self._get_max_torque(),
                                  name='max torque')
        self._verify_config_value(should=config.acceleration,
                                  actual=self._get_acceleration(),
                                  name='acceleration')
        self._verify_config_value(should=config.ramp_enable,
                                  actual=self._get_ramp_enable(),
                                  name='ramp enable')
        self._verify_config_value(should=config.target_reached_distance,
                                  actual=self._get_target_reached_distance(),
                                  name='target reached distance')
        self._verify_config_value(should=config.target_reached_velocity,
                                  actual=self._get_target_reached_velocity(),
                                  name='target reached velocity')
        self._verify_config_value(should=config.motor_halted_velocity,
                                  actual=self._get_motor_halted_velocity(),
                                  name='motor halted velocity')
        self._verify_config_value(should=config.position_p_parameter,
                                  actual=self._get_position_p_parameter(),
                                  name='position p parameter')
        self._verify_config_value(should=config.velocity_p_parameter,
                                  actual=self._get_velocity_p_parameter(),
                                  name='velocity p parameter')
        self._verify_config_value(should=config.velocity_i_parameter,
                                  actual=self._get_velocity_i_parameter(),
                                  name='velocity i parameter')
        self._verify_config_value(should=config.torque_p_parameter,
                                  actual=self._get_torque_p_parameter(),
                                  name='torque p parameter')
        self._verify_config_value(should=config.torque_i_parameter,
                                  actual=self._get_torque_i_parameter(),
                                  name='torque i parameter')
        return config

    def _verify_config_value(self, should, actual, name):
        if should != actual:
            rospy.logerr('%s: failed to configure ', self._node_name, name)
            raise InvalidConfigException()

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

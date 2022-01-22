#!/usr/bin/env python3

import yaml

from tuw_trinamic_iwos_revolute_controller.exception.invalid_file_exception import InvalidFileException
from tuw_trinamic_iwos_revolute_controller.exception.invalid_path_exception import InvalidPathException


class TrinamicConfig:
    def __init__(self):
        self.wheel_diameter = None
        self.motor_pole_pairs = None
        self.digital_hall_invert = None
        self.max_velocity = None
        self.max_torque = None
        self.acceleration = None
        self.ramp_enable = None
        self.target_reached_distance = None
        self.target_reached_velocity = None
        self.motor_halted_velocity = None
        self.position_p_parameter = None
        self.velocity_p_parameter = None
        self.velocity_i_parameter = None
        self.torque_p_parameter = None
        self.torque_i_parameter = None

    def from_file(self, config_file_path):
        config_file = self._open_config_file(config_file_path=config_file_path)
        config_content = self._read_config_file(config_file=config_file)

        self.wheel_diameter = config_content['Wheel_Diameter']
        self.motor_pole_pairs = config_content['Motor_Pole_Pairs']
        self.max_torque = config_content['Max_Torque']

        digital_hall = config_content['Digital_Hall']
        self.digital_hall_invert = digital_hall['Hall_Invert']

        linear_ramp = config_content['Linear_Ramp']
        self.max_velocity = linear_ramp['MaxVelocity']
        self.acceleration = linear_ramp['Acceleration']
        self.ramp_enable = linear_ramp['Ramp_Enabled']
        self.target_reached_distance = linear_ramp['Target_Reached_Distance']
        self.target_reached_velocity = linear_ramp['Target_Reached_Velocity']
        self.motor_halted_velocity = linear_ramp['Motor_Halted_Velocity']

        pid = config_content['PID']
        self.position_p_parameter = pid['Position_P_Parameter']
        self.velocity_p_parameter = pid['Velocity_P_Parameter']
        self.velocity_i_parameter = pid['Velocity_I_Parameter']
        self.torque_p_parameter = pid['Torque_P_Parameter']
        self.torque_i_parameter = pid['Torque_I_Parameter']

        return self

    @staticmethod
    def _open_config_file(config_file_path):
        yaml_file = open(file=config_file_path, mode='r')

        if yaml_file is None:
            raise InvalidPathException()
        else:
            return yaml_file

    @staticmethod
    def _read_config_file(config_file):
        yaml_content = yaml.load(stream=config_file, Loader=yaml.FullLoader)

        if yaml_content is None:
            raise InvalidFileException()
        else:
            return yaml_content

    def to_dynamic_reconfigure(self):
        return {
            'wheel_diameter': self.wheel_diameter,
            'motor_pole_pairs': self.motor_pole_pairs,
            'digital_hall_invert': self.digital_hall_invert,
            'max_velocity': self.max_velocity,
            'max_torque': self.max_torque,
            'acceleration': self.acceleration,
            'ramp_enable': self.ramp_enable,
            'target_reached_distance': self.target_reached_distance,
            'target_reached_velocity': self.target_reached_velocity,
            'motor_halted_velocity': self.motor_halted_velocity,
            'position_p_parameter': self.position_p_parameter,
            'velocity_p_parameter': self.velocity_p_parameter,
            'velocity_i_parameter': self.velocity_i_parameter,
            'torque_p_parameter': self.torque_p_parameter,
            'torque_i_parameter': self.torque_i_parameter,
        }

    def from_dynamic_reconfigure(self, dynamic_config):
        self.wheel_diameter = dynamic_config['wheel_diameter']
        self.motor_pole_pairs = dynamic_config['motor_pole_pairs']
        self.digital_hall_invert = dynamic_config['digital_hall_invert']
        self.max_velocity = dynamic_config['max_velocity']
        self.max_torque = dynamic_config['max_torque']
        self.acceleration = dynamic_config['acceleration']
        self.ramp_enable = dynamic_config['ramp_enable']
        self.target_reached_distance = dynamic_config['target_reached_distance']
        self.target_reached_velocity = dynamic_config['target_reached_velocity']
        self.motor_halted_velocity = dynamic_config['motor_halted_velocity']
        self.position_p_parameter = dynamic_config['position_p_parameter']
        self.velocity_p_parameter = dynamic_config['velocity_p_parameter']
        self.velocity_i_parameter = dynamic_config['velocity_i_parameter']
        self.torque_p_parameter = dynamic_config['torque_p_parameter']
        self.torque_i_parameter = dynamic_config['torque_i_parameter']

        return self

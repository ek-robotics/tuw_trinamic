#!/usr/bin/env python3

from tuw_trinamic_controller.config.abstract_comparable_config import AbstractComparableConfig
from tuw_trinamic_controller.config.abstract_default_config import AbstractDefaultConfig
from tuw_trinamic_controller.config.abstract_dynamic_config import AbstractDynamicConfig
from tuw_trinamic_controller.config.config_file_reader import ConfigFileReader


class TrinamicTMCM1640Config(AbstractComparableConfig, AbstractDefaultConfig, AbstractDynamicConfig):

    def __init__(self):
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

    def equals(self, config):
        check_list = []
        check_list += [self.true_if_equal(self.motor_pole_pairs, config.motor_pole_pairs)]
        check_list += [self.true_if_equal(self.digital_hall_invert, config.digital_hall_invert)]
        check_list += [self.true_if_equal(self.max_velocity, config.max_velocity)]
        check_list += [self.true_if_equal(self.max_torque, config.max_torque)]
        check_list += [self.true_if_equal(self.acceleration, config.acceleration)]
        check_list += [self.true_if_equal(self.ramp_enable, config.ramp_enable)]
        check_list += [self.true_if_equal(self.target_reached_distance, config.target_reached_distance)]
        check_list += [self.true_if_equal(self.target_reached_velocity, config.target_reached_velocity)]
        check_list += [self.true_if_equal(self.motor_halted_velocity, config.motor_halted_velocity)]
        check_list += [self.true_if_equal(self.position_p_parameter, config.position_p_parameter)]
        check_list += [self.true_if_equal(self.velocity_p_parameter, config.velocity_p_parameter)]
        check_list += [self.true_if_equal(self.velocity_i_parameter, config.velocity_i_parameter)]
        check_list += [self.true_if_equal(self.torque_p_parameter, config.torque_p_parameter)]
        check_list += [self.true_if_equal(self.torque_i_parameter, config.torque_i_parameter)]
        return all(check_list)
    
    @staticmethod
    def true_if_equal(value_a, value_b):
        if value_a is None:
            return True
        if value_a == value_b:
            return True
        return False

    def all_set(self):
        check_list = []
        check_list += [True if self.motor_pole_pairs is not None else False]
        check_list += [True if self.digital_hall_invert is not None else False]
        check_list += [True if self.max_velocity is not None else False]
        check_list += [True if self.max_torque is not None else False]
        check_list += [True if self.acceleration is not None else False]
        check_list += [True if self.ramp_enable is not None else False]
        check_list += [True if self.target_reached_distance is not None else False]
        check_list += [True if self.target_reached_velocity is not None else False]
        check_list += [True if self.motor_halted_velocity is not None else False]
        check_list += [True if self.position_p_parameter is not None else False]
        check_list += [True if self.velocity_p_parameter is not None else False]
        check_list += [True if self.velocity_i_parameter is not None else False]
        check_list += [True if self.torque_p_parameter is not None else False]
        check_list += [True if self.torque_i_parameter is not None else False]
        return all(check_list)

    def from_file(self, config_file_path):
        config_content = ConfigFileReader.get_config_from_file(config_file_path=config_file_path)

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

    def to_dynamic_reconfigure(self):
        return {
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

    def from_dynamic_reconfigure(self, dynamic_reconfigure):
        self.motor_pole_pairs = self._if_present(dynamic_reconfigure, 'motor_pole_pairs')
        self.digital_hall_invert = self._if_present(dynamic_reconfigure, 'digital_hall_invert')
        self.max_velocity = self._if_present(dynamic_reconfigure, 'max_velocity')
        self.max_torque = self._if_present(dynamic_reconfigure, 'max_torque')
        self.acceleration = self._if_present(dynamic_reconfigure, 'acceleration')
        self.ramp_enable = self._if_present(dynamic_reconfigure, 'ramp_enable')
        self.target_reached_distance = self._if_present(dynamic_reconfigure, 'target_reached_distance')
        self.target_reached_velocity = self._if_present(dynamic_reconfigure, 'target_reached_velocity')
        self.motor_halted_velocity = self._if_present(dynamic_reconfigure, 'motor_halted_velocity')
        self.position_p_parameter = self._if_present(dynamic_reconfigure, 'position_p_parameter')
        self.velocity_p_parameter = self._if_present(dynamic_reconfigure, 'velocity_p_parameter')
        self.velocity_i_parameter = self._if_present(dynamic_reconfigure, 'velocity_i_parameter')
        self.torque_p_parameter = self._if_present(dynamic_reconfigure, 'torque_p_parameter')
        self.torque_i_parameter = self._if_present(dynamic_reconfigure, 'torque_i_parameter')
        return self

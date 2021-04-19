#!/usr/bin/env python3

import yaml

from tuw_trinamic_controller.exception.invalid_file_exception import InvalidFileException
from tuw_trinamic_controller.exception.invalid_path_exception import InvalidPathException
from tuw_trinamic_controller.device.configuration import WheelConfiguration


class ConfigurationTool:
    """
    class providing tools for configuration management
    """
    @staticmethod
    def read_configuration(path_to_configuration):
        """
        function to read configuration from YAML file
        :param path_to_configuration: absolut path to configuration file
        :return: class containing configuration
        """
        # open yaml file
        yaml_file = open(file=path_to_configuration, mode='r')

        if yaml_file is None:
            raise InvalidPathException()

        # load yaml content
        yaml_content = yaml.load(stream=yaml_file, Loader=yaml.FullLoader)

        if yaml_content is None:
            raise InvalidFileException()

        # create configuration object
        wheel_configuration = WheelConfiguration()

        # wheel configuration
        wheel_configuration.diameter = yaml_content['Wheel_Diameter']

        # motor basic configuration
        motor_configuration = wheel_configuration.motor_configuration
        motor_configuration.pole_pairs = yaml_content['Motor_Pole_Pairs']
        motor_configuration.max_torque = yaml_content['Max_Torque']

        # motor hall configuration
        motor_hall_configuration = motor_configuration.hall_configuration
        motor_hall_configuration.hall_invert = yaml_content['Digital_Hall']['Hall_Invert']

        # motor motion configuration
        motion_configuration = motor_configuration.motion_configuration
        motion_configuration.max_velocity = yaml_content['Linear_Ramp']['MaxVelocity']
        motion_configuration.acceleration = yaml_content['Linear_Ramp']['Acceleration']
        motion_configuration.ramp_enable = yaml_content['Linear_Ramp']['Ramp_Enabled']
        motion_configuration.target_reached_velocity = yaml_content['Linear_Ramp']['Target_Reached_Velocity']
        motion_configuration.target_reached_distance = yaml_content['Linear_Ramp']['Target_Reached_Distance']
        motion_configuration.motor_halted_velocity = yaml_content['Linear_Ramp']['Motor_Halted_Velocity']

        # motor PID configuration
        motor_pid_configuration = motor_configuration.pid_configuration
        motor_pid_configuration.torque_p = yaml_content['PID']['Torque_P_Parameter']
        motor_pid_configuration.torque_i = yaml_content['PID']['Torque_I_Parameter']
        motor_pid_configuration.velocity_p = yaml_content['PID']['Velocity_P_Parameter']
        motor_pid_configuration.velocity_i = yaml_content['PID']['Velocity_I_Parameter']
        motor_pid_configuration.position_parameter = yaml_content['PID']['Position_P_Parameter']

        return wheel_configuration

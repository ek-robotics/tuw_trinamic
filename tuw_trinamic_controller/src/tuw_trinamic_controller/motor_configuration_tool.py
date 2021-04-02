#!/usr/bin/env python3

import yaml

from tuw_trinamic_controller.src.tuw_trinamic_controller.motor_configuration import MotorConfiguration


def get_motor_configuration(yaml_file_path):
    # open yaml file
    yaml_file = open(yaml_file_path, 'r')
    # load yaml content
    yaml_content = yaml.load(yaml_file, Loader=yaml.FullLoader)

    # create configuration object
    configuration = MotorConfiguration()

    # basic configuration
    configuration.pole_pairs = yaml_content['Motor_Pole_Pairs']
    configuration.max_torque = yaml_content['Max_Torque']

    # hall configuration
    configuration.hall_configuration.hall_invert = yaml_content['Digital_Hall']['Hall_Invert']

    # motion configuration
    configuration.motion_configuration.max_velocity = yaml_content['Linear_Ramp']['MaxVelocity']
    configuration.motion_configuration.acceleration = yaml_content['Linear_Ramp']['Acceleration']
    configuration.motion_configuration.ramp_enable = yaml_content['Linear_Ramp']['Ramp_Enabled']
    configuration.motion_configuration.target_reached_velocity = yaml_content['Linear_Ramp']['Target_Reached_Velocity']
    configuration.motion_configuration.target_reached_distance = yaml_content['Linear_Ramp']['Target_Reached_Distance']
    configuration.motion_configuration.motor_halted_velocity = yaml_content['Linear_Ramp']['Motor_Halted_Velocity']

    # PID configuration
    configuration.pid_configuration.torque_p = yaml_content['PID']['Torque_P_Parameter']
    configuration.pid_configuration.torque_i = yaml_content['PID']['Torque_I_Parameter']
    configuration.pid_configuration.velocity_p = yaml_content['PID']['Velocity_P_Parameter']
    configuration.pid_configuration.velocity_i = yaml_content['PID']['Velocity_I_Parameter']
    configuration.pid_configuration.position_parameter = yaml_content['PID']['Position_P_Parameter']

    return configuration


def set_motor_configuration(motor, motor_configuration):

    # motor configuration
    motor.setMotorPolePairs(motor_configuration.pole_pairs)
    motor.setMaxTorque(motor_configuration.max_torque)

    # hall sensor configuration
    motor.digitalHall.setHallInvert(motor_configuration.hall_configuration.hall_invert)

    # motion settings
    motor.linearRamp.setMaxVelocity(motor_configuration.motion_configuration.max_velocity)
    motor.linearRamp.setAcceleration(motor_configuration.motion_configuration.acceleration)
    motor.linearRamp.setRampEnabled(motor_configuration.motion_configuration.ramp_enable)
    motor.linearRamp.setTargetReachedVelocity(motor_configuration.motion_configuration.target_reached_velocity)
    motor.linearRamp.setTargetReachedDistance(motor_configuration.motion_configuration.target_reached_distance)
    motor.linearRamp.setMotorHaltedVelocity(motor_configuration.motion_configuration.motor_halted_velocity)

    # PI configuration
    motor.pid.setTorquePIParameter(motor_configuration.pid_configuration.torque_p,
                                   motor_configuration.pid_configuration.torque_i)
    motor.pid.setVelocityPIParameter(motor_configuration.pid_configuration.velocity_p,
                                     motor_configuration.pid_configuration.velocity_i)
    motor.pid.setPositionPParameter(motor_configuration.pid_configuration.position_parameter)


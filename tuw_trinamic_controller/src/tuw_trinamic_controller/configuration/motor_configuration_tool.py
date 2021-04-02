#!/usr/bin/env python3

import yaml

from tuw_trinamic_controller.configuration.motor_configuration import MotorConfiguration


def get_motor_configuration(path_to_configuration_yaml):
    # open yaml file
    yaml_file = open(file=path_to_configuration_yaml, mode='r')
    # load yaml content
    yaml_content = yaml.load(stream=yaml_file, Loader=yaml.FullLoader)

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
    motor.setMotorPolePairs(polePairs=motor_configuration.pole_pairs)
    motor.setMaxTorque(maxTorque=motor_configuration.max_torque)

    # hall sensor configuration
    motor.digitalHall.setHallInvert(invert=motor_configuration.hall_configuration.hall_invert)

    # motion settings
    motor.linearRamp.setMaxVelocity(maxVelocity=motor_configuration.motion_configuration.max_velocity)
    motor.linearRamp.setAcceleration(acceleration=motor_configuration.motion_configuration.acceleration)
    motor.linearRamp.setRampEnabled(enable=motor_configuration.motion_configuration.ramp_enable)
    motor.linearRamp.setTargetReachedVelocity(velocity=motor_configuration.motion_configuration.target_reached_velocity)
    motor.linearRamp.setTargetReachedDistance(distance=motor_configuration.motion_configuration.target_reached_distance)
    motor.linearRamp.setMotorHaltedVelocity(velocity=motor_configuration.motion_configuration.motor_halted_velocity)

    # PI configuration
    motor.pid.setTorquePIParameter(pValue=motor_configuration.pid_configuration.torque_p,
                                   iValue=motor_configuration.pid_configuration.torque_i)
    motor.pid.setVelocityPIParameter(pValue=motor_configuration.pid_configuration.velocity_p,
                                     iValue=motor_configuration.pid_configuration.velocity_i)
    motor.pid.setPositionPParameter(pValue=motor_configuration.pid_configuration.position_parameter)


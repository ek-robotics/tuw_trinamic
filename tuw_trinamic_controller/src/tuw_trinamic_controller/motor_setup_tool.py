#!/usr/bin/env python

from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.modules.TMCM1640.TMCM_1640 import TMCM_1640

from tuw_trinamic_controller.src.tuw_trinamic_controller.motor_configuration_tool import get_motor_configuration
from tuw_trinamic_controller.src.tuw_trinamic_controller.motor_configuration_tool import set_motor_configuration


def setup_motor_with_configuration(path_to_configuration_yaml):
    module_connection = ConnectionManager().connect()
    module = TMCM_1640(connection=module_connection)

    motor = module.motor(0)
    motor_configuration = get_motor_configuration(path_to_configuration_yaml=path_to_configuration_yaml)
    set_motor_configuration(motor=motor, motor_configuration=motor_configuration)

    return motor


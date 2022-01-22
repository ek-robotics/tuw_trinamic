#!/usr/bin/env python3

import rospy

from tuw_trinamic_iwos_revolute_controller.connection.trinamic_connection import TrinamicConnection
from tuw_trinamic_iwos_revolute_controller.exception.invalid_config_exception import InvalidConfigException
from tuw_trinamic_iwos_revolute_controller.exception.invalid_file_exception import InvalidFileException
from tuw_trinamic_iwos_revolute_controller.exception.invalid_path_exception import InvalidPathException


class ConnectionHandler:
    def __init__(self):
        self._node_name = rospy.get_name()
        self._connections = {}

    def connect(self, usb_ports):
        log_string = '{node_name}: ATTEMPTING TO SETUP:'.format(node_name=self._node_name)
        log_string += ''.join(['\n - wheel on port {port}'.format(port=port) for port in usb_ports.values()])
        rospy.loginfo(log_string)

        for usb_port_key, usb_port in usb_ports.items():
            self._connections[usb_port_key] = self._connect_trinamic(usb_port)

    @staticmethod
    def _connect_trinamic(usb_port):
        try:
            trinamic_connection = TrinamicConnection(usb_port=usb_port)
        except InvalidPathException:
            rospy.logerr('failed to load configuration (invalid path)')
        except InvalidFileException:
            rospy.logerr('failed to load configuration (invalid file)')
        except ConnectionError:
            rospy.logwarn('failed to connect to device on USB port %s', usb_port)
        else:
            rospy.loginfo('succeeded to connect to device on USB port %s', usb_port)
            return trinamic_connection

    def set_config(self, config):
        for connection in self._connections:
            connection.set_config(config)

        return self._check_configs_identical()

    def fetch_config(self):
        return self._check_configs_identical()

    def set_target_velocity_rpm(self, target_velocities_rpm):
        for usb_port_key, target_velocity_rpm in target_velocities_rpm.items():
            self._connections[usb_port_key].set_target_velocity_rpm(target_velocity_rpm=target_velocity_rpm)

    def set_target_velocity(self, target_velocities):
        for usb_port_key, target_velocity in target_velocities.items():
            self._connections[usb_port_key].set_target_velocity(target_velocity=target_velocity)

    def _check_configs_identical(self):
        configs = [(connection.fetch_config() for connection in self._connections)]

        if all(config == configs[0] for config in configs):
            return configs[0]
        else:
            log_string = str(self._node_name) + ': the config for the devices is not identical over all devices'
            rospy.logerr()
            raise InvalidConfigException

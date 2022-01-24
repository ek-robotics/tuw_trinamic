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

    def connect(self, usb_ports, attempts=10):
        log_string = '{node_name}: ATTEMPTING TO SETUP:'.format(node_name=self._node_name)
        log_string += ''.join(['\n - wheel on port {port}'.format(port=port) for port in usb_ports.values()])
        rospy.loginfo(log_string)

        self._connections = {key: None for key in usb_ports}

        for attempt in range(1, attempts+1):
            rospy.loginfo('%s: connecting (%2d of %2d)', self._node_name, attempt, attempts)
            for usb_port in usb_ports:
                self._connections['usb_port'] = self._connect_trinamic(usb_port)

            if all(self._connections):
                break
            else:
                rospy.sleep(1)

        if None in self._connections.values():
            rospy.logerr('shutting down node ...')
            rospy.signal_shutdown('failed to connect to wheel(s)')

    @staticmethod
    def _connect_trinamic(usb_port):
        try:
            trinamic_connection = TrinamicConnection(usb_port=usb_port)
        except InvalidPathException:
            rospy.logerr('failed to load configuration (invalid path)')
            return None
        except InvalidFileException:
            rospy.logerr('failed to load configuration (invalid file)')
            return None
        except ConnectionError:
            rospy.logwarn('failed to connect to device on USB port %s', usb_port)
            return None
        else:
            rospy.loginfo('succeeded to connect to device on USB port %s', usb_port)
            return trinamic_connection

    def set_config(self, config):
        for connection in self._connections:
            connection.set_config(config)

        return self._check_configs_identical()

    def fetch_config(self):
        return self._check_configs_identical()

    def set_target_velocity_rpm(self, target_velocity_rpm_dict):
        for usb_port_key, target_velocity_rpm in target_velocity_rpm_dict.items():
            self._connections[usb_port_key].set_target_velocity_rpm(target_velocity_rpm=target_velocity_rpm)

    def set_target_velocity(self, target_velocity_dict):
        for usb_port_key, target_velocity in target_velocity_dict.items():
            self._connections[usb_port_key].set_target_velocity(target_velocity=target_velocity)

    def _check_configs_identical(self):
        configs = [(connection.fetch_config() for connection in self._connections)]

        if all(config == configs[0] for config in configs):
            return configs[0]
        else:
            rospy.logerr('%s: the config is not consistent over all devices', self._node_name)
            raise InvalidConfigException

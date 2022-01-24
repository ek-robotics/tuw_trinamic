#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

from tuw_trinamic_iwos_revolute_controller.connection.trinamic_connection import TrinamicConnection
from tuw_trinamic_iwos_revolute_controller.exception.invalid_config_exception import InvalidConfigException
from tuw_trinamic_iwos_revolute_controller.exception.invalid_file_exception import InvalidFileException
from tuw_trinamic_iwos_revolute_controller.exception.invalid_path_exception import InvalidPathException


class ConnectionHandler:
    def __init__(self):
        self._node_name = rospy.get_name()
        self._index = {'left': 0, 'right': 1}
        self._directions = {'left': 1, 'right': 1}
        self._connections = {'left': None, 'right': None}

    def connect(self, usb_ports, attempts=10):
        log_string = '{node_name}: ATTEMPTING TO SETUP:'.format(node_name=self._node_name)
        log_string += ''.join(['\n - wheel on port {port}'.format(port=port) for port in usb_ports.values()])
        rospy.loginfo(log_string)

        for attempt in range(1, attempts + 1):
            rospy.loginfo('%s: connecting (%2d of %2d)', self._node_name, attempt, attempts)

            self._connections['left'] = self._connect_trinamic(usb_ports[0])
            self._connections['right'] = self._connect_trinamic(usb_ports[1])

            if all(self._connections):
                break
            else:
                rospy.sleep(1)

        if not all(self._connections):
            rospy.logerr('shutting down node ...')
            rospy.signal_shutdown('failed to connect to wheel(s)')

    def swap_connections(self):
        self._connections['left'], self._connections['right'] = self._connections['right'], self._connections['left']

    def change_direction(self, side):
        self._directions[side] *= -1

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
        for connection in self._connections.values():
            connection.set_config(config)

        return self._check_configs_identical()

    def verify_config(self):
        return self._check_configs_identical()

    def set_target_velocity(self, target_velocities):
        for side, index in self._index.items():
            target_velocity = target_velocities[index] * self._directions[side]
            self._connections[side].set_target_velocity(target_velocity=target_velocity)

    def get_state(self):
        state_list = [connection.get_state(name='{side}_revolute'.format(side=side))
                      for side, connection in self._connections.items()]

        merged_state = JointState()
        merged_state.name = [joint_state.name for joint_state in state_list]
        merged_state.position = [joint_state.position for joint_state in state_list]
        merged_state.velocity = [joint_state.velocity for joint_state in state_list]
        merged_state.effort = [joint_state.effort for joint_state in state_list]
        return merged_state

    def _check_configs_identical(self):
        configs = [(connection.get_config() for connection in self._connections.values())]

        if all(config == configs[0] for config in configs):
            return configs[0]
        else:
            rospy.logerr('%s: the config is not consistent over all devices', self._node_name)
            raise InvalidConfigException

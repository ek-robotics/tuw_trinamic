#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from threading import Lock

from tuw_trinamic_iwos_revolute_controller.connection.revolute_connection import RevoluteConnection
from tuw_trinamic_iwos_revolute_controller.exception.invalid_config_exception import InvalidConfigException
from tuw_trinamic_iwos_revolute_controller.exception.invalid_file_exception import InvalidFileException
from tuw_trinamic_iwos_revolute_controller.exception.invalid_path_exception import InvalidPathException


class ConnectionHandler:
    def __init__(self):
        self.lock = Lock()
        self._node_name = rospy.get_name()
        self._index = {'left': 0, 'right': 1}
        self._directions = {'left': 1, 'right': 1}
        self._connections = {'left': None, 'right': None}
        self._state_seq = 0

    def connect(self, usb_ports, attempts=10):
        rospy.loginfo('%s: ATTEMPTING TO SETUP PORTS: %s, %s', self._node_name, usb_ports[0], usb_ports[1])

        for attempt in range(1, attempts + 1):
            rospy.loginfo('%s: connecting (attempt %2d of %2d)', self._node_name, attempt, attempts)

            self._create_connection(side='left', usb_port=usb_ports[0])
            self._create_connection(side='right', usb_port=usb_ports[1])

            if all(self._connections.values()):
                break
            else:
                rospy.sleep(1)

        if not all(self._connections.values()):
            rospy.signal_shutdown('failed to connect to wheel(s)')

    def _create_connection(self, side, usb_port):
        self.lock.acquire()
        try:
            self._connections[side] = RevoluteConnection(usb_port=usb_port)
        except InvalidPathException:
            rospy.logerr('%s: failed to load configuration (invalid path)', self._node_name)
            self._connections[side] = None
        except InvalidFileException:
            rospy.logerr('%s: failed to load configuration (invalid file)', self._node_name)
            self._connections[side] = None
        except ConnectionError:
            rospy.logwarn('%s: failed to connect to device on USB port %s', self._node_name, usb_port)
            self._connections[side] = None
        else:
            rospy.loginfo('%s: succeeded to connect to device on USB port %s', self._node_name, usb_port)
        self.lock.release()

    def set_config(self, config):
        self.lock.acquire()
        for connection in self._connections.values():
            connection.set_config(config)
        self.lock.release()

        return self.verify_config()

    def verify_config(self):
        self.lock.acquire()
        configs = [(connection.get_config() for connection in self._connections.values())]
        self.lock.release()

        if not all(config == configs[0] for config in configs):
            rospy.logerr('%s: the config is not consistent over all devices', self._node_name)
            raise InvalidConfigException

        return configs[0]

    def set_target_velocity(self, target_velocities):
        self.lock.acquire()
        for side, index in self._index.items():
            target_velocity = target_velocities[index] * self._directions[side]
            self._connections[side].set_target_velocity(target_velocity=target_velocity)
        self.lock.release()

    def get_state(self):
        self.lock.acquire()
        state_list = [connection.get_state(name='{side}_revolute'.format(side=side))
                      for side, connection in self._connections.items()]

        merged_state = JointState()
        merged_state.header.seq = self._state_seq
        merged_state.header.stamp = rospy.get_rostime()
        merged_state.header.frame_id = 'revolute'
        for joint_state in state_list:
            merged_state.name.extend(joint_state.name)
            merged_state.position.extend(joint_state.position)
            merged_state.velocity.extend(joint_state.velocity)
            merged_state.effort.extend(joint_state.effort)
        self._state_seq += 1
        self.lock.release()
        return merged_state

    def change_direction(self, side):
        self._directions[side] *= -1

    def exchange_connections(self):
        self._connections['left'], self._connections['right'] = self._connections['right'], self._connections['left']
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

from tuw_trinamic_controller.exception.invalid_config_exception import InvalidConfigException
from tuw_trinamic_controller.exception.invalid_file_exception import InvalidFileException
from tuw_trinamic_controller.exception.invalid_path_exception import InvalidPathException


class ConnectionHandler:

    def __init__(self, connection_type, config_type):
        self._node_name = rospy.get_name()
        self._connection_type = connection_type
        self._config_type = config_type
        self._ports = {}
        self._state_sequence = 0

    def connect_ports(self, ports, attempts=10):
        rospy.loginfo('%s: connection on %d ports', self._node_name, len(self._ports))
        self._ports = {port: None for port in ports}

        for attempt in range(0, attempts):
            for port, connection in self._ports.items():
                if connection is None:
                    self.connect_port(port=port, attempt=attempt, attempts=attempts)

            if all(self._ports.values()):
                break
            else:
                rospy.sleep(1)

        if not all(self._ports.values()):
            log_string = '{node_name}: failed to connect'.format(node_name=self._node_name)
            rospy.logerr(log_string)
            rospy.signal_shutdown(log_string)

    def connect_port(self, port, attempt, attempts):
        try:
            self._ports[port] = self._connection_type(port=port, config_type=self._config_type)
        except ConnectionError:
            rospy.logwarn('%s: failed to connect to device on USB port %s (attempt %2d of %2d)',
                          self._node_name, port, attempt + 1, attempts + 1)
            self._ports[port] = None
        else:
            rospy.loginfo('%s: succeeded to connect to device on USB port %s (attempt %2d of %2d)',
                          self._node_name, port, attempt + 1, attempts + 1)

    def set_config_from_file(self, config_file_path):
        try:
            config = self._config_type().from_file(config_file_path=config_file_path)
            return self.set_config(config)
        except InvalidPathException:
            rospy.logerr('%s: failed to load config (invalid path)', self._node_name)
        except InvalidFileException:
            rospy.logerr('%s: failed to load config (invalid file)', self._node_name)

    def set_config(self, config):
        if config is None:
            rospy.logerr('%s: config is invalid', self._node_name)
            raise InvalidConfigException

        config_list = [port.set_config(config=config) for port in self._ports.values()]
        if not all(config_list):
            rospy.logerr('%s: config set faulty', self._node_name)
            raise InvalidConfigException

        check_list = [config_list[0].equals(config) for config in config_list]
        if not all(check_list):
            rospy.logerr('%s: config inconsistent', self._node_name)
            raise InvalidConfigException

        return config

    def callback_reconfigure(self, dynamic_reconfigure, level):
        config = self._config_type().from_dynamic_reconfigure(dynamic_reconfigure=dynamic_reconfigure)
        return self.set_config(config=config).to_dynamic_reconfigure()

    def callback_command(self, command):
        for port, port_command in zip(self._ports.items(), command):
            port.callback_command(command=port_command)

    def get_state(self):
        state_list = [port.get_state() for port in self._ports.items()]
        return JointState(
            name=[state.name for state in state_list],
            position=[state.position for state in state_list],
            velocity=[state.velocity for state in state_list],
            effort=[state.effort for state in state_list]
        )

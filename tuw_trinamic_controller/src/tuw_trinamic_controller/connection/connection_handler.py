#!/usr/bin/env python3

import rospy
from tuw_nav_msgs.msg import Joints

from tuw_trinamic_controller.exception.invalid_config_exception import InvalidConfigException
from tuw_trinamic_controller.exception.invalid_file_exception import InvalidFileException
from tuw_trinamic_controller.exception.invalid_path_exception import InvalidPathException


class ConnectionHandler:

    def __init__(self, connection_type, config_type):
        self._node_name = rospy.get_name()
        self._connection_type = connection_type
        self._config_type = config_type
        self._config = None
        self._devices = {}
        self._state_sequence = 0

    def connect_devices(self, ports, baudrate, attempts=10):
        rospy.loginfo('%s: connecting on ports %s, %s (baudrate: %s)', self._node_name, ports[0], ports[1], baudrate)
        self._devices = {port: None for port in ports}

        for attempt in range(1, attempts + 1):
            for port, connection in self._devices.items():
                if connection is None:
                    self.connect_device(port=port, baudrate=baudrate, attempt=attempt, attempts=attempts)

            if all(self._devices.values()):
                break
            else:
                rospy.sleep(1)

        if not all(self._devices.values()):
            log_string = '{node_name}: failed to connect'.format(node_name=self._node_name)
            rospy.logerr(log_string)
            rospy.signal_shutdown(log_string)

    def connect_device(self, port, baudrate, attempt, attempts):
        try:
            self._devices[port] = self._connection_type(port=port, baudrate=baudrate, config_type=self._config_type)
        except ConnectionError:
            rospy.logwarn('%s: failed to connect to device on USB port %s (attempt %2d of %2d)',
                          self._node_name, port, attempt, attempts)
            self._devices[port] = None
        else:
            rospy.loginfo('%s: succeeded to connect to device on USB port %s (attempt %2d of %2d)',
                          self._node_name, port, attempt, attempts)

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

        config_list = [device.set_config(config=config) for device in self._devices.values()]
        if not all(config_list):
            rospy.logerr('%s: config set faulty', self._node_name)
            raise InvalidConfigException

        check_list = [config_list[0].equals(config) for config in config_list[1:]]
        if not all(check_list):
            rospy.logerr('%s: config inconsistent', self._node_name)
            raise InvalidConfigException

        self._config = config_list[0]
        return config

    def get_config(self):
        return self._config

    def callback_reconfigure(self, dynamic_reconfigure, level):
        if level == -1:
            return self.get_config().to_dynamic_reconfigure()

        config = self._config_type().from_dynamic_reconfigure(dynamic_reconfigure=dynamic_reconfigure)
        return self.set_config(config=config).to_dynamic_reconfigure()

    def callback_command(self, command_message):
        for device, command in zip(self._devices.values(), self._joints_message_to_list(message=command_message)):
            device.set_command(command=command)

    def _joints_message_to_list(self, message):
        commands = []
        for index, device in enumerate(self._devices.values()):
            commands.append(
                Joints(
                    header=message.header,
                    position=[list(message.position)[index] if len(message.position) - 1 >= index else None],
                    velocity=[list(message.velocity)[index] if len(message.velocity) - 1 >= index else None],
                    torque=[list(message.torque)[index] if len(message.torque) - 1 >= index else None]))
        return commands

    def get_state(self):
        state_list = [device.get_state() for device in self._devices.values()]
        return Joints(
            position=[state['position'] for state in state_list],
            velocity=[state['velocity'] for state in state_list],
            torque=[state['torque'] for state in state_list]
        )

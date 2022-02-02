#!/usr/bin/env python3

from abc import ABC, abstractmethod


class AbstractTrinamicConnection(ABC):

    @abstractmethod
    def __init__(self, port, baudrate, config_type):
        """
        constructor
        :param port: port of the connection
        :param baudrate: baudrate of the connection
        :param config_type: config type of the device
        """
        pass

    @abstractmethod
    def set_command(self, command):
        """
        set command to device
        :param command:  command to set
        :return:
        """
        pass

    @abstractmethod
    def get_state(self):
        """
        get joint state from device
        :return: joint state from device
        """
        pass

    @abstractmethod
    def set_config(self, config, verify=True):
        """
        set config on device and verify config is set correct
        config is set correct if all values not None are set correct
        :param config: config to set
        :param verify: boolean flag to decide if config shall be verified after setting
        :return: verify True: config on device (if set successful) or None (if not set successful)
                 verify False: config on device
        """
        pass

    @abstractmethod
    def get_config(self):
        """
        get config from device
        :return: config from device
        """
        pass

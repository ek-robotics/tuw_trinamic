#!/usr/bin/env python3

from abc import ABC, abstractmethod


class AbstractTrinamicConnection(ABC):

    @abstractmethod
    def __init__(self, port, config_type):
        """
        constructor
        :param port: port of the device
        :param config_type: config type of the device
        """
        pass

    @abstractmethod
    def set_command(self, command):
        """
        set command to device
        :param command:  command to set
        :return: True (if command was set successful) or False (if command was not set successful)
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
    def set_config(self, config):
        """
        set config on device and verify config is set correct (by comparing config to set with config from get)
        :param config: config to set
        :return: config to set (if set successful) or None (if not set successful)
        """
        pass

    @abstractmethod
    def get_config(self):
        """
        get config from device
        :return: config from device
        """
        pass

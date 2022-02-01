#!/usr/bin/env python3

from abc import ABC, abstractmethod


class AbstractDefaultConfig(ABC):

    @abstractmethod
    def from_file(self, config_file_path):
        """
        set all config values based on config file (yaml)
        :param config_file_path: path to config file (yaml)
        :return:
        """
        pass

    @staticmethod
    def _if_present(config_content, key):
        if key in config_content:
            return config_content[key]
        if key not in config_content:
            return None

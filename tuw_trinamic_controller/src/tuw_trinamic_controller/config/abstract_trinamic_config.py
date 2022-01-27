#!/usr/bin/env python3

from abc import ABC, abstractmethod

from tuw_trinamic_controller.config.abstract_default_config import AbstractDefaultConfig
from tuw_trinamic_controller.config.abstract_dynamic_config import AbstractDynamicConfig


class AbstractTrinamicConfig(AbstractDefaultConfig, AbstractDynamicConfig):

    @abstractmethod
    def equals(self, config):
        """
        check if all values of config are equal
        :param config: config to check if equal
        :return: True (if equal) or False (if not equal)
        """
        pass

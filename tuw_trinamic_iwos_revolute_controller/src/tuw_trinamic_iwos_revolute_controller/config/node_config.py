#!/usr/bin/env python3

from tuw_trinamic_iwos_revolute_controller.config.abstract_dynamic_config import AbstractDynamicConfig
from tuw_trinamic_iwos_revolute_controller.config.abstract_file_config import AbstractFileConfig
from tuw_trinamic_iwos_revolute_controller.file_handler.file_handler import FileHandler


class NodeConfig(AbstractDynamicConfig,):
    def __init__(self):
        self.reverse_left_wheel = False
        self.reverse_right_wheel = False
        self.swap_wheels = False

    def from_file(self, config_file_path):
        # TODO: create function to read node config from file
        pass

    def to_dynamic_reconfigure(self):
        return {
            'reverse_left_wheel': self.reverse_left_wheel,
            'reverse_right_wheel': self.reverse_right_wheel,
            'swap_wheels': self.swap_wheels,
        }

    def from_dynamic_reconfigure(self, dynamic_reconfigure):
        self.reverse_left_wheel = dynamic_reconfigure['reverse_left_wheel']
        self.reverse_right_wheel = dynamic_reconfigure['reverse_right_wheel']
        self.swap_wheels = dynamic_reconfigure['swap_wheels']

        return self

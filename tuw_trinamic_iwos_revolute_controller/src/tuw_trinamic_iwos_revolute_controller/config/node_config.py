#!/usr/bin/env python3

from tuw_trinamic_iwos_revolute_controller.config.abstract.default_config import DefaultConfig
from tuw_trinamic_iwos_revolute_controller.config.abstract.dynamic_config import DynamicConfig
from tuw_trinamic_iwos_revolute_controller.file_handler.file_handler import FileHandler


class NodeConfig(DefaultConfig, DynamicConfig):
    def __init__(self):
        self.reverse_left_wheel = False
        self.reverse_right_wheel = False
        self.exchange_wheels = False
        self.state_rate = 0

    def from_file(self, config_file_path):
        config_file = FileHandler.open_config_file(config_file_path=config_file_path)
        config_content = FileHandler.read_config_file(config_file=config_file)

        self.reverse_left_wheel = config_content['Reverse_Left_Wheel']
        self.reverse_right_wheel = config_content['Reverse_Right_Wheel']
        self.exchange_wheels = config_content['Exchange_Wheels']
        self.state_rate = config_content['State_Rate']

        return self

    def to_dynamic_reconfigure(self):
        return {
            'reverse_left_wheel': self.reverse_left_wheel,
            'reverse_right_wheel': self.reverse_right_wheel,
            'exchange_wheels': self.exchange_wheels,
            'state_rate': self.state_rate
        }

    def from_dynamic_reconfigure(self, dynamic_config):
        self.reverse_left_wheel = dynamic_config['reverse_left_wheel']
        self.reverse_right_wheel = dynamic_config['reverse_right_wheel']
        self.exchange_wheels = dynamic_config['exchange_wheels']
        self.state_rate = dynamic_config['state_rate']
        return self

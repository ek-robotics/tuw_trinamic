#!/usr/bin/env python3

class NodeConfig:
    def __init__(self):
        self.reverse_left_wheel = None
        self.reverse_right_wheel = None
        self.swap_wheels = None

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

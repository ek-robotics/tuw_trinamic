#!/usr/bin/env python3

import rospy


class ConfigHandler:
    def __init__(self, node_config, device_config):
        self._node_name = rospy.get_name()
        self.node_config = node_config
        self.device_config = device_config

    def set_device_config(self, connection_handler):
        connection_handler.set_config(config=self.device_config)

    def fetch_device_config(self, connection_handler):
        self.device_config = connection_handler.fetch_config()

    def dynamic_reconfigure_callback_device(self, dynamic_reconfigure):
        pass

    def dynamic_reconfigure_callback_node(self, dynamic_reconfigure):
        pass

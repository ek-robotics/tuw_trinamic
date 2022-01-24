#!/usr/bin/env python3

import rospy


class ConfigHandler:
    def __init__(self, node_connection, node_config, revolute_connection, revolute_config):
        self._node_name = rospy.get_name()
        self.node_connection = node_connection
        self.node_config = node_config
        self.revolute_connection = revolute_connection
        self.revolute_config = revolute_config

    def set_node_config(self):
        self.node_connection.set_config(config=self.node_config)

    def set_revolute_config(self):
        self.revolute_connection.set_config(config=self.revolute_config)

    def fetch_revolute_config(self):
        self.revolute_config = self.revolute_connection.fetch_config()

    def dynamic_reconfigure_callback_node(self, dynamic_reconfigure, level):
        if level == -1:
            return self.node_config.to_dynamic_reconfigure()

        self.node_config.from_dynamic_reconfigure(dynamic_reconfigure=dynamic_reconfigure)
        self.set_node_config()
        return self.node_config.to_dynamic_reconfigure()

    def dynamic_reconfigure_callback_revolute(self, dynamic_reconfigure, level):
        if level == -1:
            return self.revolute_config.to_dynamic_reconfigure()

        self.revolute_config.from_dynamic_reconfigure(dynamic_reconfigure=dynamic_reconfigure)
        self.set_revolute_config()
        self.fetch_revolute_config()
        return self.revolute_config.to_dynamic_reconfigure

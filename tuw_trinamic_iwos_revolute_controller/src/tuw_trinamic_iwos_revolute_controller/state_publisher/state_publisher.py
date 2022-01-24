#!/usr/bin/env python3

import rospy


class StatePublisher:
    def __init__(self, rate, publisher, connection_handler, config_handler):
        self._rate = self.set_rate(rate)
        self._publisher = publisher
        self.connection_handler = connection_handler
        self.config_handler = config_handler

    def run(self):
        while not rospy.is_shutdown():
            self._publish_state(state=self._fetch_state())
            self._rate.sleep()

    def _fetch_state(self):
        return self.connection_handler.get_state()

    def _publish_state(self, state):
        self._publisher.publish(state)

    def set_rate(self, rate):
        self._rate = rospy.Rate(int(rate))
        return self._rate

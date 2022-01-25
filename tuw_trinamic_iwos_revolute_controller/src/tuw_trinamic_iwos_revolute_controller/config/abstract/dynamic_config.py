#!/usr/bin/env python3

from abc import ABC, abstractmethod


class DynamicConfig(ABC):
    @abstractmethod
    def to_dynamic_reconfigure(self):
        pass

    @abstractmethod
    def from_dynamic_reconfigure(self, dynamic_config):
        pass

#!/usr/bin/env python3

from abc import ABC, abstractmethod


class DefaultConfig(ABC):
    @abstractmethod
    def from_file(self, config_file_path):
        pass

#!/usr/bin/env python3

from abc import ABC, abstractmethod


class AbstractFileConfig(ABC):
    @abstractmethod
    def from_file(self, config_file_path):
        pass

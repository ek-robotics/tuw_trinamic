#!/usr/bin/env python3

from abc import ABC, abstractmethod


class AbstractComparableConfig(ABC):

    @abstractmethod
    def equals(self, config):
        """
        check if all values of config are equal
        :param config: config to check if equal
        :return: True (if equal) or False (if not equal)
        """
        pass

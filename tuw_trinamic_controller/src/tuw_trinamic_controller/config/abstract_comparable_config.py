#!/usr/bin/env python3

from abc import ABC, abstractmethod


class AbstractComparableConfig(ABC):

    @abstractmethod
    def equals(self, config):
        """
        check if all values set (not None) are equal
        :param config: config to check if equal
        :return: True (if equal) or False (if not equal)
        """
        pass

    @staticmethod
    def _true_if_equal(value_a, value_b):
        if value_a is None:
            return True
        if value_a == value_b:
            return True
        return False


    @abstractmethod
    def all_set(self):
        """
        check if all values are set (not None)
        :return: True (if all set) or False (if not all set)
        """
        pass

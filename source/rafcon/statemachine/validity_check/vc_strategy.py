"""
.. module:: vc_strategy
   :platform: Unix, Windows
   :synopsis: A module implementing a strategy pattern for choosing the correct validity check strategy

.. moduleauthor:: Sebastian Brunner


"""

from enum import Enum

Strategy = Enum('ValidityChecker', 'DEFAULT NONE LIGHT AGGRESSIVE')


class VCStrategy:
    """A base class for collecting useful functionality, that is needed by all validity check implementations

    """

    def __init__(self):
        pass

    def check(self, container_state):
        """ A interface functions that has to be implemented by a concrete validity checker strategy.

        :param container_state:
        :return:
        """
        raise NotImplementedError("The check() function has to be implemented!")

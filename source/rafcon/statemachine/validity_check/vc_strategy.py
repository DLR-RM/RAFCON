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
        raise NotImplementedError("The check() function has to be implemented!")

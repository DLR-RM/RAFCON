"""
.. module:: singleton
   :platform: Unix, Windows
   :synopsis: A module holding a generic singleton abstract class

.. moduleauthor:: Franz Steimetz


"""


class Singleton(type):
    """Meta class for classes implementing the singleton pattern

    The meta class ensures that the class can only be instantiated once. Usage:

    .. code-block:: python
        class Logger(object):
        __metaclass__ = Singleton

        Or in Python3

        class Logger(metaclass=Singleton):
            pass
    """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

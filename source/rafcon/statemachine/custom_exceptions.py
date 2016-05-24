"""
.. module:: custom_exceptions
   :platform: Unix, Windows
   :synopsis: A module that holds all custom exceptions for the state machine core

.. moduleauthor:: Sebastian Brunner


"""


class LibraryNotFoundException(Exception):

    def __init__(self, message):
        """ A custom exception for the case when a library is not found

        :param message: the error message for the exception
        :return:
        """

        # Call the base class constructor with the parameters it needs
        super(LibraryNotFoundException, self).__init__(message)

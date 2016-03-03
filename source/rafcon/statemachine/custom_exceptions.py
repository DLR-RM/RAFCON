"""
.. module:: library_manager
   :platform: Unix, Windows
   :synopsis: A module that holds all custom exceptions for the state machine core

.. moduleauthor:: Sebastian Brunner


"""


class LibraryNotFoundException(Exception):

    def __init__(self, message):

        # Call the base class constructor with the parameters it needs
        super(LibraryNotFoundException, self).__init__(message)
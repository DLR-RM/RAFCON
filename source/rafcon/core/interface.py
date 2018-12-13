# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: interface
   :synopsis: This is a interface for user input. In absence of a GUI the input is read from stdin.
                If e.g. a gtk GUI is implemented the function can be replaced with appropriate dialog boxes.

"""

from builtins import input
import os


def open_folder_cmd_line(query, default_path=None):
    """Queries the user for a path to open
    
    :param str query: Query that asks the user for a specific folder path to be opened
    :param str default_path: Path to use if the user doesn't specify a path 
    :return: Input path from the user or `default_path` if nothing is specified or None if path does not exist
    :rtype: str
    """
    user_input = input(query + ': ')
    if len(user_input) == 0:
        user_input = default_path
    if not user_input or not os.path.isdir(user_input):
        return None
    return user_input

open_folder_func = open_folder_cmd_line


def create_folder_cmd_line(query, default_name=None, default_path=None):
    """Queries the user for a path to be created

    :param str query: Query that asks the user for a specific folder path  to be created
    :param str default_name: Default name of the folder to be created
    :param str default_path: Path in which the folder is created if the user doesn't specify a path
    :return: Input path from the user or `default_path` if nothing is specified or None if directory could ne be created
    :rtype: str
    """
    default = None
    if default_name and default_path:
        default = os.path.join(default_path, default_name)
    user_input = input(query + ' [default {}]: '.format(default))
    if len(user_input) == 0:
        user_input = default

    if not user_input:
        return None
    if not os.path.isdir(user_input):
        try:
            os.makedirs(user_input)
        except OSError:
            return None
    return user_input

create_folder_func = create_folder_cmd_line


def save_folder_cmd_line(query, default_name=None, default_path=None):
    """Queries the user for a path or file to be saved into

    The folder or file has not to be created already and will not be created by this function. The parent directory
    of folder and file has to exist otherwise the function will return None.

    :param str query: Query that asks the user for a specific folder/file path to be created
    :param str default_name: Default name of the folder to be created
    :param str default_path: Path in which the folder is created if the user doesn't specify a path
    :return: Input path from the user or `default_path` if nothing is specified and None if directory does not exist
    :rtype: str
    """
    default = None
    if default_name and default_path:
        default = os.path.join(default_path, default_name)
    user_input = input(query + ' [default {}]: '.format(default))
    if len(user_input) == 0:
        user_input = default

    if not user_input or not os.path.isdir(os.path.dirname(user_input)):
        return None
    return user_input

save_folder_func = save_folder_cmd_line


def show_notice(notice):
    """Shows a notice on the console that has to be acknowledged 
    
    :param str notice: Notice to show to the user 
    """
    input(notice)

show_notice_func = show_notice

# Copyright (C) 2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>

"""
.. module:: resources
   :synopsis: A module for accessing non-code resource files, like style or config files

"""

import sys
import os
from os.path import expanduser, join, dirname, abspath


possible_prefix_paths = []
share_folder_paths = []
installation_share_folder = None

try:
    from gi.repository import GLib
    xdg_user_data_folder = GLib.get_user_data_dir()
except ImportError:
    xdg_user_data_folder = os.getenv("XDG_DATA_HOME", join(expanduser("~"), ".local", "share"))
    pass

# The list is ordered by preference
_possible_prefix_paths = [sys.prefix, sys.exec_prefix,
                          os.getenv("VIRTUAL_ENV"), os.getenv("PYTHONUSERBASE"),
                          dirname(xdg_user_data_folder), join(os.sep, "usr", "local"), join(os.sep, "usr")]

# Check which paths exist and remove duplicates
for prefix_path in _possible_prefix_paths:
    if prefix_path and prefix_path not in possible_prefix_paths and os.path.isdir(prefix_path):
        possible_prefix_paths.append(prefix_path)
        share_folder_path = join(prefix_path, "share")
        share_folder_paths.append(share_folder_path)
        if not installation_share_folder and os.access(share_folder_path, os.W_OK):
            installation_share_folder = share_folder_path


def get_repository_share_path():
    """Get the share folder from the repository

    If started from repository, the path to the share folder within the repository is returned, otherwise, `None`.
    """
    try:
        import rafcon
        share_path = join(dirname(dirname(dirname(abspath(rafcon.__file__)))), "share")
        if os.path.isdir(share_path):
            return share_path
        return None
    except ImportError:
        return None


def get_data_file_path(*rel_path):
    """Get a file installed as data_file (located in share folder)"""
    relative_share_path = get_repository_share_path()
    if relative_share_path:
        return os.path.join(relative_share_path, *rel_path)
    return search_in_share_folders(*rel_path)


def search_in_share_folders(*rel_path):
    """Search all possible share folders for a given resource"""
    for share_folder_path in share_folder_paths:
        abs_resource_path = os.path.join(share_folder_path, *rel_path)
        if os.path.exists(abs_resource_path):
            return abs_resource_path

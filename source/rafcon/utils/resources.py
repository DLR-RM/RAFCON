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
from os import listdir
from os.path import expanduser, isfile, join
import pkg_resources


possible_prefix_paths = []
share_folder_paths = []
installation_share_folder = None

# The list is ordered by preference
_possible_prefix_paths = [sys.prefix, sys.exec_prefix,
                          os.getenv("VIRTUAL_ENV"), os.getenv("PYTHONUSERBASE"),
                          join(expanduser("~"), ".local"), join(os.sep, "usr", "local"), join(os.sep, "usr")]

# Check which paths exist and remove duplicates
for prefix_path in _possible_prefix_paths:
    if prefix_path and prefix_path not in possible_prefix_paths and os.path.isdir(prefix_path):
        possible_prefix_paths.append(prefix_path)
        share_folder_path = join(prefix_path, "share")
        share_folder_paths.append(share_folder_path)
        if not installation_share_folder and os.access(share_folder_path, os.W_OK):
            installation_share_folder = share_folder_path

# One more candidate for share folder: GLib.get_user_data_dir()
try:
    from gi.repository import GLib
    user_data_folder = GLib.get_user_data_dir()
    if user_data_folder not in share_folder_paths:
        share_folder_paths.insert(min(2, len(share_folder_paths)), user_data_folder)
except ImportError:
    pass


def resource_filename(package_or_requirement, resource_name):
    """
    Similar to pkg_resources.resource_filename but if the resource it not found via pkg_resources
    it also looks in a predefined list of paths in order to find the resource

    :param package_or_requirement: the module in which the resource resides
    :param resource_name: the name of the resource
    :return: the path to the resource
    :rtype: str
    """
    if pkg_resources.resource_exists(package_or_requirement, resource_name):
        return pkg_resources.resource_filename(package_or_requirement, resource_name)

    path = search_in_share_folders(package_or_requirement, resource_name)

    if path:
        return path

    raise RuntimeError("Resource {} not found in {}".format(package_or_requirement, resource_name))


def resource_exists(package_or_requirement, resource_name):
    """
    Similar to pkg_resources.resource_exists but if the resource it not found via pkg_resources
    it also looks in a predefined list of paths in order to find the resource

    :param package_or_requirement: the module in which the resource resides
    :param resource_name: the name of the resource
    :return: a flag if the file exists
    :rtype: bool
    """
    if pkg_resources.resource_exists(package_or_requirement, resource_name):
        return True

    path = search_in_share_folders(package_or_requirement, resource_name)

    return True if path else False


def resource_string(package_or_requirement, resource_name):
    """
    Similar to pkg_resources.resource_string but if the resource it not found via pkg_resources
    it also looks in a predefined list of paths in order to find the resource

    :param package_or_requirement: the module in which the resource resides
    :param resource_name: the name of the resource
    :return: the file content
    :rtype: str
    """
    with open(resource_filename(package_or_requirement, resource_name), 'r') as resource_file:
        return resource_file.read()


def resource_listdir(package_or_requirement, relative_path):
    """
    Similar to pkg_resources.resource_listdir but if the resource it not found via pkg_resources
    it also looks in a predefined list of paths in order to find the resource

    :param package_or_requirement: the module in which the resource resides
    :param relative_path: the relative path to the resource
    :return: a list of all files residing in the target path
    :rtype: list
    """
    path = resource_filename(package_or_requirement, relative_path)
    only_files = [f for f in listdir(path) if isfile(join(path, f))]
    return only_files


def search_in_share_folders(package_or_requirement, resource_name):
    package_or_requirement_segments = package_or_requirement.split(".")

    # go up the whole module path and search for the module
    # i.e. if a resource "r" is searched in rafcon.gui.controllers,
    # the resource "r" residing in rafcon.gui may also be returned
    while package_or_requirement_segments:
        paths = list(map(
            lambda path: os.path.join(os.path.join(path, *package_or_requirement_segments), resource_name),
            share_folder_paths,
        ))

        for path in paths:
            if os.path.isfile(path) or os.path.isdir(path):
                return path

        package_or_requirement_segments.pop()

    return None

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
from past.builtins import map
import os
from os import listdir
from os.path import expanduser, isfile, join
import pkg_resources


share_folder_paths = []

possible_prefix_paths = [sys.prefix, sys.exec_prefix,
                         os.getenv("PYTHONUSERBASE"), os.getenv("VIRTUAL_ENV"),
                         join(expanduser("~"), ".local"), join(os.sep, "usr", "local"), join(os.sep, "usr")]

for prefix_path in possible_prefix_paths:
    if prefix_path:
        prefix_path = join(prefix_path, "share")
        if prefix_path not in share_folder_paths:
            share_folder_paths.append(prefix_path)


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

    path = _search_in_share_folders(package_or_requirement, resource_name)

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

    path = _search_in_share_folders(package_or_requirement, resource_name)

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


def _search_in_share_folders(package_or_requirement, resource_name):
    package_or_requirement_segments = package_or_requirement.split(".")

    # go up the whole module path and search for the module
    # i.e. if a resource "r" is searched in rafcon.gui.controllers,
    # the resource "r" residing in rafcon.gui may also be returned
    while package_or_requirement_segments:
        paths = map(
            lambda path: os.path.join(os.path.join(path, *package_or_requirement_segments), resource_name),
            share_folder_paths,
        )

        for path in paths:
            if os.path.isfile(path) or os.path.isdir(path):
                return path

        package_or_requirement_segments.pop()

    return None

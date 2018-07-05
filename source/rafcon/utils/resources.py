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

import os
from os import listdir
from os.path import expanduser, isfile, join
import pkg_resources


paths_to_search_for_resource = [
    join(expanduser("~"), ".local", "share"),
    join('usr', 'local', 'share'),
    join('usr', 'share'),
]

# Check for non-default PYTHONUSERBASE and add it to the seach path list
pythonuserbase = os.getenv("PYTHONUSERBASE")
if pythonuserbase:
    pythonuserbase = join(pythonuserbase, "share")
    if pythonuserbase not in paths_to_search_for_resource:
        paths_to_search_for_resource.append(pythonuserbase)


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

    package_or_requirement_os = os.path.join(package_or_requirement.split("."))

    # go up the whole module path and search for the module
    # i.e. if a resource "r" is searched in rafcon.gui.controllers,
    # the resource "r" residing in rafcon.gui may also be returned
    while not package_or_requirement_os == []:
        paths = map(
            lambda path: os.path.join(os.path.join(path, *package_or_requirement_os), resource_name),
            paths_to_search_for_resource,
        )

        for path in paths:
            if os.path.isfile(path):
                return path
            if os.path.isdir(path):
                return path

        package_or_requirement_os = package_or_requirement_os[:-1]

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

    package_or_requirement_os = os.path.join(package_or_requirement.split("."))

    # go up the whole module path and search for the module
    # i.e. if a resource "r" is searched in rafcon.gui.controllers,
    # the resource "r" residing in rafcon.gui may also be returned
    while not package_or_requirement_os == []:
        paths = map(
            lambda path: os.path.join(os.path.join(path, *package_or_requirement_os), resource_name),
            paths_to_search_for_resource,
        )

        for path in paths:
            if os.path.isfile(path):
                return True
            if os.path.isdir(path):
                return True
        package_or_requirement_os = package_or_requirement_os[:-1]

    return False


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


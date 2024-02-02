# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: filesystem
   :synopsis: A module to hold all filesystem specific utility functions

"""
import os
import tarfile
import stat

from os.path import join, expanduser
from rafcon.core.config import global_config

import shutil, errno


def create_path(path):
    """Creates a absolute path in the file system.

    :param path: The path to be created
    """
    import os
    if not os.path.exists(path):
        os.makedirs(path)


def read_file(file_path, filename=None):
    """ Open file by path and optional filename

    If no file name is given the path is interpreted as direct path to the file to be read.
    If there is no file at location the return value will be None to offer a option for case handling.

    :param str file_path: Path string.
    :param str filename: File name of the file to be read.
    :return: None or str
    """
    file_path = os.path.realpath(file_path)
    if filename:
        file_path = os.path.join(file_path, filename)

    file_content = None
    if os.path.isfile(file_path):
        with open(file_path, 'r') as file_pointer:
            file_content = file_pointer.read()

    return file_content

    
def write_file(file_path, content, create_full_path=False):
    file_path = os.path.realpath(file_path)
    if create_full_path:
        head, tail = os.path.split(file_path)
        create_path(head)
    with open(file_path, 'w') as file_pointer:
        file_pointer.write(content)
    
        
def get_default_config_path():
    home_path = expanduser('~')
    if home_path:
        return join(home_path, ".config", "rafcon")
    return None


def get_default_log_path():
    default_path = None
    home_path = expanduser('~')
    if home_path:
        default_path = join(home_path, ".log", "rafcon")
    return global_config.get_config_value("DEFAULT_LOG_PATH", default_path)


def separate_folder_path_and_file_name(path):
    if os.path.isdir(path):
        return path, None
    else:
        return os.path.split(path)


def clean_file_system_paths_from_not_existing_paths(file_system_paths):
    """Cleans list of paths from elements that do not exist

    If a path is no more valid/existing, it is removed from the list.

    :param list[str] file_system_paths: list of file system paths to be checked for existing
    """
    paths_to_delete = []
    for path in file_system_paths:
        if not os.path.exists(path):
            paths_to_delete.append(path)
    for path in paths_to_delete:
        file_system_paths.remove(path)


def make_tarfile(output_filename, source_dir):
    with tarfile.open(output_filename, "w:gz") as tar:
        tar.add(source_dir, arcname=os.path.basename(source_dir))


def copy_file_or_folder(src, dst):
    try:
        shutil.copytree(src, dst)
    except OSError as exc:
        if exc.errno == errno.ENOTDIR:
            shutil.copy(src, dst)
        else:
            raise


def make_file_executable(filename):
    st = os.stat(filename)
    os.chmod(filename, st.st_mode | stat.S_IEXEC)


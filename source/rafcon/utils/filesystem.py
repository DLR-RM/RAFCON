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
import shutil
from os.path import realpath, dirname, join, expanduser


def create_path(path):
    """Creates a absolute path in the file system.

    :param path: The path to be created
    """
    import os
    if not os.path.exists(path):
        os.makedirs(path)


def get_md5_file_hash(filename):
    """Calculates the MD5 hash of a file

    :param str filename: The filename (including the path) of the file
    :return: Md5 hash of the file
    :rtype: str
    """
    import hashlib
    BLOCKSIZE = 65536
    hasher = hashlib.md5()
    with open(filename, 'rb') as afile:
        buf = afile.read(BLOCKSIZE)
        while len(buf) > 0:
            hasher.update(buf)
            buf = afile.read(BLOCKSIZE)
    return hasher.hexdigest()


def file_needs_update(target_file, source_file):
    """Checks if target_file is not existing or differing from source_file

    :param target_file: File target for a copy action
    :param source_file: File to be copied
    :return: True, if target_file not existing or differing from source_file, else False
    :rtype: False
    """
    if not os.path.isfile(target_file) or get_md5_file_hash(target_file) != get_md5_file_hash(source_file):
        return True
    return False


def copy_file_if_update_required(source_file, target_file):
    """Copies source_file to target_file if latter one in not existing or outdated

    :param source_file: Source file of the copy operation
    :param target_file: Target file of the copy operation
    """
    if file_needs_update(target_file, source_file):
        shutil.copy(source_file, target_file)


def read_file(file_path, filename=None):
    file_path = os.path.realpath(file_path)
    if filename:
        file_path = os.path.join(file_path, filename)

    file_content = ""
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
    
        
def get_home_path():
    home_path = expanduser('~')
    if home_path:
        return join(home_path, ".config", "rafcon")
    return None


def read_version_from_pt_file():
    import rafcon
    pt_file_name = 'rafcon.pt'
    pt_file_path = join(dirname(dirname(dirname(realpath(rafcon.__file__)))), pt_file_name)
    with open(pt_file_path) as pt_file:
        for line in pt_file:
            if line.strip().startswith('VERSION'):
                parts = line.split('=')
                version = parts[1].strip()
                return version
    return 0


def separate_folder_path_and_file_name(path):
    if os.path.isdir(path):
        return path, None
    else:
        return os.path.split(path)

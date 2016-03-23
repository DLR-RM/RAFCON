"""
.. module:: filesystem
   :platform: Unix, Windows
   :synopsis: A module to hold all filesystem specific utility functions

.. moduleauthor:: Franz Steinmetz


"""


import os
import shutil


def create_path(path):
    """Creates a absolute path in the file system.

    :param path: The path to be created
    """
    import os
    if not os.path.exists(path):
        os.makedirs(path)


def remove_path(path):
    """Removes an absolute path in the file system

    :param path: The path to be removed
    """
    import shutil
    shutil.rmtree(path)


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


def write_file(file_path, content):
    file_path = os.path.realpath(file_path)
    with open(file_path, 'w') as file_pointer:
        file_pointer.write(content)

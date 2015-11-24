
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
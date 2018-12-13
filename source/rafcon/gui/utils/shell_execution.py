import shlex
import subprocess

from rafcon.utils import log
_logger = log.get_logger(__name__)


def execute_command_with_path_in_process(command, path, shell=False, cwd=None, logger=None):
    """Executes a specific command in a separate process with a path as argument.

    :param command: the command to be executed
    :param path: the path as first argument to the shell command
    :param bool shell: Whether to use a shell
    :param str cwd: The working directory of the command
    :param logger: optional logger instance which can be handed from other module
    :return: None
    """
    if logger is None:
        logger = _logger
    logger.debug("Opening path with command: {0} {1}".format(command, path))
    # This splits the command in a matter so that the command gets called in a separate shell and thus
    # does not lock the window.
    args = shlex.split('{0} "{1}"'.format(command, path))
    try:
        subprocess.Popen(args, shell=shell, cwd=cwd)
        return True
    except OSError as e:
        logger.error('The operating system raised an error: {}'.format(e))
    return False


def execute_command_in_process(command, shell=False, cwd=None, logger=None):
    """ Executes a specific command in a separate process

    :param command: the command to be executed
    :param bool shell: Whether to use a shell
    :param str cwd: The working directory of the command
    :param logger: optional logger instance which can be handed from other module
    :return: None
    """
    if logger is None:
        logger = _logger
    logger.debug("Run shell command: {0}".format(command))
    try:
        subprocess.Popen(command, shell=shell, cwd=cwd)
        return True
    except OSError as e:
        logger.error('The operating system raised an error: {}'.format(e))
    return False

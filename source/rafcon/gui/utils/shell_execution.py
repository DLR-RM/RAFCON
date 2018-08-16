import shlex
import subprocess

from rafcon.utils import log
_logger = log.get_logger(__name__)


def execute_shell_command_with_file_path(command, path, logger=None):
        """ Executes a specific command in the shell (in our case an editor).

        :param command: the command to be executed
        :param path: the path as first argument to the shell command
        :param logger: optional logger instance which can be handed from other module
        :return: None
        """
        if logger is None:
            logger = _logger
        logger.debug("Opening path with command: {}".format(command))
        # This splits the command in a matter so that the editor gets called in a separate shell and thus
        # does not lock the window.
        args = shlex.split('{0} "{1}"'.format(command, path))
        try:
            subprocess.Popen(args)
            return True
        except OSError as e:
            logger.error('The operating system raised an error: {}'.format(e))
        return False
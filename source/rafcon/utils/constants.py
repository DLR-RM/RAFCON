"""
.. module:: constants
   :platform: Unix, Windows
   :synopsis: A module holding all constants of the RAFCON core

.. moduleauthor:: Sebastian Brunner


"""

import tempfile
import getpass
import os
import stat

from rafcon.core.config import global_config, CONFIG_FILE


TEMP_PATH = global_config.get_config_value("TEMP_PATH")
TEMP_PATH = tempfile.gettempdir() if TEMP_PATH is None else TEMP_PATH
RAFCON_TEMP_PATH_BASE = os.path.join(TEMP_PATH, 'rafcon-{0}'.format(getpass.getuser()), str(os.getpid()))

# check if the given temp-folder is read and writable -> TODO most probably needs adaptation for windows -> test it
if not (bool(os.stat(TEMP_PATH).st_mode & stat.S_IRUSR) and bool(os.stat(TEMP_PATH).st_mode & stat.S_IWUSR) or
        os.stat(TEMP_PATH).st_uid == os.getuid()):
    raise OSError("The given or the default temp-folder {0} is not read and writable please use the variable TEMP_PATH "
                  "in the {1} file to adjust the path to one which fulfill this criteria.".format(TEMP_PATH, CONFIG_FILE))

# if global_config.get_config_value("TEMP_PATH") is None:
#     global_config.set_config_value("TEMP_PATH", TEMP_PATH)
#     global_config.save_configuration()

try:
    os.makedirs(RAFCON_TEMP_PATH_BASE)
except OSError:  # Raised when directory is already existing, thus can be ignored
    pass

RAFCON_TEMP_PATH_STORAGE = tempfile.mkdtemp(dir=RAFCON_TEMP_PATH_BASE)

BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS = ['state_execution_status']

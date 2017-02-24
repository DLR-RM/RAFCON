# Copyright

"""
.. module:: constants
   :synopsis: A module holding all constants of the RAFCON core

"""

import tempfile
import getpass
import os
import stat

TEMP_PATH = tempfile.gettempdir()
RAFCON_TEMP_PATH_BASE = os.path.join(TEMP_PATH, 'rafcon-{0}'.format(getpass.getuser()), str(os.getpid()))

# check if the given temp-folder is read and writable
if not (bool(os.stat(TEMP_PATH).st_mode & stat.S_IRUSR) and bool(os.stat(TEMP_PATH).st_mode & stat.S_IWUSR) or
        os.stat(TEMP_PATH).st_uid == os.getuid()):
    raise OSError("The given or the default tmp-folder '{0}' has to be read and writable please use the environment "
                  "variables TMPDIR, TEMP and TMP (in this order) to adjust the path to one which fulfill this criteria."
                  "".format(TEMP_PATH))

try:
    os.makedirs(RAFCON_TEMP_PATH_BASE)
except OSError:  # Raised when directory is already existing, thus can be ignored
    pass

RAFCON_TEMP_PATH_STORAGE = tempfile.mkdtemp(dir=RAFCON_TEMP_PATH_BASE)

BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS = ['state_execution_status']

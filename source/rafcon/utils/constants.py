import tempfile
import getpass
import os


RAFCON_TEMP_PATH_BASE = os.path.join(tempfile.gettempdir(), 'rafcon-{0}/{1}'.format(getpass.getuser(), os.getpid()))

try:
    os.makedirs(RAFCON_TEMP_PATH_BASE)
except OSError:  # Raised when directory is already existing, thus can be ignored
    pass

RAFCON_TEMP_PATH_STORAGE = tempfile.mkdtemp(dir=RAFCON_TEMP_PATH_BASE)


import sys
import gtk
import logging
from utils import log
from models import StateModel
from controllers import StatePropertiesController
from views import StatePropertiesView
from statemachine import State


def setup_path():
    """Sets up the python include paths to include needed directories"""
    import os.path
    import sys

    #sys.path.insert(1, '.')
    #sys.path.insert(0, reduce(os.path.join, (TOPDIR, "resources", "external")))
    #sys.path.insert(0, os.path.join(TOPDIR, "src"))
    return


def check_requirements():
    """Checks versions and other requirements"""
    import gtkmvc
    gtkmvc.require("1.99.1")
    return


def main(*args, **kargs):
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)
    logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)

    my_state = State()
    prop_model = StateModel(my_state)
    prop_view = StatePropertiesView()
    prop_ctrl = StatePropertiesController(prop_model, prop_view)
    my_state.name = "test2"
    logger.debug("changed attribute")
    gtk.main()
    logger.debug("after gtk main")

    return

if __name__ == "__main__":
    setup_path()
    check_requirements()
    main()
    pass

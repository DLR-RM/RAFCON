
import sys
import gtk
import logging
from utils import log
from models import StateModel, ContainerStateModel
from controllers import StatePropertiesController, ContainerStateController
from views import StatePropertiesView, ContainerStateView
from views.transition_list import TransitionListView
from statemachine import State, ContainerState, Transition


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

    state1 = State('State1')
    state2 = State('State2')
    state3 = State('State3')
    # prop_model = StateModel(my_state)
    # prop_view = StatePropertiesView()
    # prop_ctrl = StatePropertiesController(prop_model, prop_view)
    #
    trans1 = Transition(state1.state_id, 1, state2.state_id, 2)
    trans2 = Transition(state2.state_id, 1, state3.state_id, 3)
    ctr_state = ContainerState(states=[state1, state2, state3], transitions=[trans1, trans2])
    ctr_state.name = "Container"
    ctr_model = ContainerStateModel(ctr_state)
    # prop_view2 = StatePropertiesView()
    # prop_ctrl2 = StatePropertiesController(prop_model2, prop_view2)
    #
    # my_state.name = "test2"
    # my_state2.name = "ContainerState"
    # logger.debug("changed attribute")

    ctr_view = ContainerStateView()
    # for i in iter(ctr_view):
    #     print i
    ContainerStateController(ctr_model, ctr_view)
    #TransitionListView()
    gtk.main()
    logger.debug("after gtk main")

    return

if __name__ == "__main__":
    setup_path()
    check_requirements()
    main()
    pass

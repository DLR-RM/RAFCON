
import sys
import gtk
import logging
from utils import log
from mvc.models import StateModel, ContainerStateModel
from mvc.controllers import StatePropertiesController, ContainerStateController, GraphicalEditorController
from mvc.views import StatePropertiesView, ContainerStateView, GraphicalEditorView
from mvc.views.transition_list import TransitionListView
from statemachine.states.state import State
from statemachine.states.container_state import ContainerState
from statemachine.transition import Transition
from statemachine.data_flow import DataFlow


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
    state3 = ContainerState(name='State3')
    print "State3 ID", state3.state_id
    state4 = State('Nested')
    state3.add_state(state4)
    state1.add_outcome('Success')
    state3.add_outcome('Branch1')
    state3.add_outcome('Branch2')

    # prop_model = StateModel(my_state)
    # prop_view = StatePropertiesView()
    # prop_ctrl = StatePropertiesController(prop_model, prop_view)
    #
    trans1 = Transition(state1.state_id, 1, state2.state_id, 2)
    trans2 = Transition(state2.state_id, 1, state3.state_id, 3)
    data_flow1 = DataFlow(state1.state_id, "success", state2.state_id, None)
    data_flow2 = DataFlow(state2.state_id, "success", state3.state_id, None)
    ctr_state = ContainerState(name="Container")
    print "Ctr ID", ctr_state.state_id
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    ctr_state.add_transition(state1.state_id, 3, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, 2, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, 2, None, 2)
    ctr_state.add_transition(state1.state_id, 1, None, 1)
    # ctr_state.transitions = [trans1, trans2]
    # ctr_state.data_flows = [data_flow1, data_flow2]
        # states=[state1, state2, state3], transitions=[trans1, trans2], data_flows=[data_flow1,
        #                                                                                                   data_flow2])
    ctr_state.name = "Container"

    ctr_model = ContainerStateModel(ctr_state)
    print "Ctr", len(ctr_state.states), "state3", len(state3.states)
    # prop_view2 = StatePropertiesView()
    # prop_ctrl2 = StatePropertiesController(prop_model2, prop_view2)
    #
    # my_state.name = "test2"
    # my_state2.name = "ContainerState"
    # logger.debug("changed attribute")

    #ctr_view = ContainerStateView()

    #ContainerStateController(ctr_model, ctr_view)

    editor_view = GraphicalEditorView()
    editor_ctrl = GraphicalEditorController(ctr_model, editor_view)

    gtk.main()
    logger.debug("after gtk main")

    return

if __name__ == "__main__":
    setup_path()
    check_requirements()
    main()
    pass

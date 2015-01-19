
import sys
import gtk
import logging
from utils import log
from mvc.models import StateModel, ContainerStateModel
from mvc.controllers import StatePropertiesController, ContainerStateController, GraphicalEditorController,\
    StateDataPortEditorController
from mvc.views import StatePropertiesView, ContainerStateView, GraphicalEditorView, StateDataportEditorView
from mvc.views.transition_list import TransitionListView
from statemachine.states.state import State, DataPort
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
    state1.add_output_data_port("output", "int")
    state1.add_input_data_port("input", "int", 0)
    state2 = State('State2')
    state2.add_input_data_port("my_input", "int", 0)
    state2.add_input_data_port("longlonginputname", "int", 0)
    state2.add_input_data_port("par", "int", 0)
    state2.add_output_data_port("my_output", "int")
    state2.add_output_data_port("res", "int")
    state3 = ContainerState(name='State3')
    state3.add_input_data_port("input", "int", 0)
    state3.add_output_data_port("output", "int")
    state4 = State('Nested')
    state4.add_output_data_port("out", "int")
    state5 = State('Nested2')
    state5.add_input_data_port("in", "int", 0)
    state3.add_state(state4)
    state3.add_state(state5)
    state3.add_transition(state4.state_id, 0, state5.state_id, None)
    state3.add_data_flow(state4.state_id, "out", state5.state_id, "in")
    state1.add_outcome('Success')
    state3.add_outcome('Branch1')
    state3.add_outcome('Branch2')

    ctr_state = ContainerState(name="Container")
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    ctr_state.add_input_data_port("ctr_in", "int", 0)
    ctr_state.add_output_data_port("ctr_out", "int")
    ctr_state.add_transition(state1.state_id, 0, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, -2, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, -2, None, -2)
    ctr_state.add_transition(state1.state_id, -1, None, -1)
    ctr_state.add_data_flow(state1.state_id, "output", state2.state_id, "par")
    ctr_state.add_data_flow(state2.state_id, "res", state3.state_id, "input")
    ctr_state.add_data_flow(ctr_state.state_id, "ctr_in", state1.state_id, "input")
    ctr_state.add_data_flow(state3.state_id, "output", ctr_state.state_id, "ctr_out")
    ctr_state.name = "Container"
    # TODO: me
    ctr_state.add_input_data_port("input_data1", "str", "default_value1")
    ctr_state.add_input_data_port("input_data2", "str", "default_value2")
    ctr_state.add_input_data_port("input_data3", "str", "default_value3")

    ctr_state.add_output_data_port("output_data1", "str", "default_value1")
    ctr_state.add_output_data_port("output_data2", "str", "default_value2")
    ctr_state.add_output_data_port("output_data3", "str", "default_value3")

    ctr_state.add_scoped_variable("scoped_variable1", "str", "default_value1")
    ctr_state.add_scoped_variable("scoped_variable2", "str", "default_value1")
    ctr_state.add_scoped_variable("scoped_variable3", "str", "default_value1")

    ctr_model = ContainerStateModel(ctr_state)
    # prop_view2 = StatePropertiesView()
    # prop_ctrl2 = StatePropertiesController(prop_model2, prop_view2)
    #
    # my_state.name = "test2"
    # my_state2.name = "ContainerState"
    # logger.debug("changed attribute")

    # TODO: me
    sdev = StateDataportEditorView()
    sdec = StateDataPortEditorController(ctr_model, sdev)
    #ctr_model.input_data_ports.append("test")
    #ctr_model.state_input_data_ports["test_name"] = DataPort("test_name", "str", "test_default_value")
    ctr_model.state.name = "New_name"
    #ctr_model.state.input_data_ports = {}
    ctr_model.state.add_input_data_port("test_name", "str", "test_default_value")

    #input_data_port_observer = InputDataPortListObserver()
    #input_data_port_observer.observe_model(ctr_model)

    gtk.main()
    return

    ctr_view = ContainerStateView()

    ContainerStateController(ctr_model, ctr_view)

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

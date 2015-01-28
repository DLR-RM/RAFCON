from statemachine.states.state import DataPort, DataPortType
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.library_state import LibraryState
from states.execution_state import ExecutionState
from statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState

from state_machine_manager import StateMachineManager
from external_modules.external_module import ExternalModule
from statemachine.storage.storage import Storage

import statemachine.singleton


def concurrency_barrier_test():
    state1 = ExecutionState("FirstState", path="../../test_scripts", filename="concurrence_barrier1.py")
    state1.add_outcome("FirstOutcome", 3)
    input_state1 = state1.add_input_data_port("FirstDataInputPort", "str")
    output_state1 = state1.add_output_data_port("FirstDataOutputPort", "float")

    state2 = ExecutionState("SecondState", path="../../test_scripts", filename="concurrence_barrier2.py")
    state2.add_outcome("FirstOutcome", 3)
    input_state2 = state2.add_input_data_port("FirstDataInputPort", "str")
    output_state2 = state2.add_output_data_port("FirstDataOutputPort", "float")

    state3 = BarrierConcurrencyState("FirstConcurrencyState", path="../../test_scripts",
                                     filename="concurrency_container.py")
    state3.add_state(state1)
    input_state3 = state3.add_input_data_port("in1", "str", "default_in1_string")
    input2_state3 = state3.add_input_data_port("in2", "str", "default_in2_string")
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_state(state2)
    state3.add_data_flow(state3.state_id, input2_state3, state2.state_id, input_state2)
    state3.add_output_data_port("out1", "str", "default_output_value")

    input_data = {"in1": "in1_string", "in2": "in2_string"}
    output_data = {"out1": None}
    state3.input_data = input_data
    state3.output_data = output_data
    state3.start()
    state3.join()


def concurrency_barrier_save_load_test():
    s = Storage("../../test_scripts/stored_statemachine")
    state1 = ExecutionState("FirstState", path="../../test_scripts", filename="concurrence_barrier1.py")
    state1.add_outcome("FirstOutcome", 3)
    input_state1 = state1.add_input_data_port("FirstDataInputPort", "str")
    output_state1 = state1.add_output_data_port("FirstDataOutputPort", "float")

    state2 = ExecutionState("SecondState", path="../../test_scripts", filename="concurrence_barrier2.py")
    state2.add_outcome("FirstOutcome", 3)
    input_state2 = state2.add_input_data_port("FirstDataInputPort", "str")
    output_state2 = state2.add_output_data_port("FirstDataOutputPort", "float")

    state3 = BarrierConcurrencyState("FirstConcurrencyState", path="../../test_scripts",
                                     filename="concurrency_container.py")
    state3.add_state(state1)
    input_state3 = state3.add_input_data_port("in1", "str", "default_in1_string")
    input2_state3 = state3.add_input_data_port("in2", "str", "default_in2_string")
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_state(state2)
    state3.add_data_flow(state3.state_id, input2_state3, state2.state_id, input_state2)
    state3.add_output_data_port("out1", "str", "default_output_value")

    s.save_statemachine_as_yaml(state3)
    root_state = s.load_statemachine_from_yaml()

    input_data = {"in1": "in1_string", "in2": "in2_string"}
    output_data = {"out1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    root_state.start()
    root_state.join()


def concurrency_preemption_test():
    state1 = ExecutionState("FirstState", path="../../test_scripts", filename="concurrence_preemption1.py")
    state1.add_outcome("FirstOutcome", 3)
    input_state1 = state1.add_input_data_port("FirstDataInputPort", "str")
    output_state1 = state1.add_output_data_port("FirstDataOutputPort", "float")

    state2 = ExecutionState("SecondState", path="../../test_scripts", filename="concurrence_preemption2.py")
    state2.add_outcome("FirstOutcome", 3)
    input_state2 = state2.add_input_data_port("FirstDataInputPort", "str")
    output_state2 = state2.add_output_data_port("FirstDataOutputPort", "float")

    state3 = PreemptiveConcurrencyState("FirstConcurrencyState", path="../../test_scripts",
                                     filename="concurrency_container.py")
    state3.add_state(state1)
    input_state3 = state3.add_input_data_port("in1", "str", "default_in1_string")
    input2_state3 = state3.add_input_data_port("in2", "str", "default_in2_string")
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_state(state2)
    state3.add_data_flow(state3.state_id, input2_state3, state2.state_id, input_state2)
    state3.add_output_data_port("out1", "str", "default_output_value")

    input_data = {"in1": "in1_string", "in2": "in2_string"}
    output_data = {"out1": None}
    state3.input_data = input_data
    state3.output_data = output_data
    state3.start()
    state3.join()


def concurrency_preemption_save_load_test():
    s = Storage("../../test_scripts/stored_statemachine")
    state1 = ExecutionState("FirstState", path="../../test_scripts", filename="concurrence_preemption1.py")
    state1.add_outcome("FirstOutcome", 3)
    input_state1 = state1.add_input_data_port("FirstDataInputPort", "str")
    output_state1 = state1.add_output_data_port("FirstDataOutputPort", "float")

    state2 = ExecutionState("SecondState", path="../../test_scripts", filename="concurrence_preemption2.py")
    state2.add_outcome("FirstOutcome", 3)
    input_state2 = state2.add_input_data_port("FirstDataInputPort", "str")
    output_state2 = state2.add_output_data_port("FirstDataOutputPort", "float")

    state3 = PreemptiveConcurrencyState("FirstConcurrencyState", path="../../test_scripts",
                                     filename="concurrency_container.py")
    state3.add_state(state1)
    input_state3 = state3.add_input_data_port("in1", "str", "default_in1_string")
    input2_state3 = state3.add_input_data_port("in2", "str", "default_in2_string")
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_state(state2)
    state3.add_data_flow(state3.state_id, input2_state3, state2.state_id, input_state2)
    state3.add_output_data_port("out1", "str", "default_output_value")


    s.save_statemachine_as_yaml(state3)
    root_state = s.load_statemachine_from_yaml()

    input_data = {"in1": "in1_string", "in2": "in2_string"}
    output_data = {"out1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    root_state.start()
    root_state.join()


def hierarchy_test():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="first_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("MyFirstDataInputPort", "str")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    state3 = HierarchyState("MyFirstHierarchyState", path="../../test_scripts", filename="hierarchy_container.py")
    state3.add_state(state1)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, None, 6)
    state3.add_input_data_port("in1", "str")
    state3.add_input_data_port("in2", "int")
    state3.add_output_data_port("out1", "str")
    state3.add_data_flow(state3.state_id,
                         state3.get_io_data_port_id_from_name_and_type("in1", DataPortType.INPUT),
                         state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("MyFirstDataInputPort", DataPortType.INPUT))
    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None}
    state3.input_data = input_data
    state3.output_data = output_data
    state3.start()
    #time.sleep(1.0)
    #state3.preempted = True
    state3.join()
    #print "joined thread"


def hierarchy_save_load_test():
    s = Storage("../../test_scripts/stored_statemachine")

    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="first_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("MyFirstDataInputPort", "str")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    state3 = HierarchyState("MyFirstHierarchyState", path="../../test_scripts", filename="hierarchy_container.py")
    state3.add_state(state1)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, None, 6)
    state3.add_input_data_port("in1", "str")
    state3.add_input_data_port("in2", "int")
    state3.add_output_data_port("out1", "str")
    state3.add_data_flow(state3.state_id,
                         state3.get_io_data_port_id_from_name_and_type("in1", DataPortType.INPUT),
                         state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("MyFirstDataInputPort", DataPortType.INPUT))

    s.save_statemachine_as_yaml(state3)
    root_state = s.load_statemachine_from_yaml()

    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    root_state.start()
    root_state.join()


def global_variable_test():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="global_variable_test_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    input_state1 = state1.add_input_data_port("MyFirstDataInputPort", "str")
    output_state1 = state1.add_output_data_port("MyFirstDataOutputPort", "float")

    state3 = HierarchyState("MyFirstHierarchyState", path="../../test_scripts", filename="hierarchy_container.py")
    state3.add_state(state1)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, None, 6)
    input_state3 = state3.add_input_data_port("in1", "str")
    input2_state3 = state3.add_input_data_port("in2", "int")
    output_state2 = state3.add_output_data_port("out1", "str")
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None}
    state3.input_data = input_data
    state3.output_data = output_data
    state3.start()
    state3.join()


def state_machine_manager_test():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="first_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("MyFirstDataInputPort", "str")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    input_data = {"MyFirstDataInputPort": "input_string"}
    output_data = {"MyFirstDataOutputPort": None}
    state1.input_data = input_data
    state1.output_data = output_data

    sm = StateMachineManager(state1)
    sm.start()


def external_modules_test():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="external_module_test_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("MyFirstDataInputPort", "str")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    input_data = {"MyFirstDataInputPort": "input_string"}
    output_data = {"MyFirstDataOutputPort": None}
    state1.input_data = input_data
    state1.output_data = output_data

    em = ExternalModule(name="em1", module_name="external_module_test", class_name="TestModule")
    statemachine.singleton.external_module_manager.add_external_module(em)
    statemachine.singleton.external_module_manager.external_modules["em1"].connect([])
    statemachine.singleton.external_module_manager.external_modules["em1"].start()

    sm = StateMachineManager(state1)
    sm.start()


def ros_external_module_test():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="ros_test_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("MyFirstDataInputPort", "str")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    input_data = {"MyFirstDataInputPort": "input_string"}
    output_data = {"MyFirstDataOutputPort": None}
    state1.input_data = input_data
    state1.output_data = output_data

    em = ExternalModule(name="ros", module_name="ros_external_module", class_name="RosModule")
    statemachine.singleton.external_module_manager.add_external_module(em)
    statemachine.singleton.external_module_manager.external_modules["ros"].connect([])
    statemachine.singleton.external_module_manager.external_modules["ros"].start()

    sm = StateMachineManager(state1)
    sm.start()


def save_and_load_data_port_test():
    s = Storage("../../test_scripts/stored_statemachine")
    data_port1 = DataPort("test", "str")
    s.save_file_as_yaml_rel(data_port1, "saved_data_port")
    loaded_data_port = s.load_file_from_yaml_rel("saved_data_port")
    print loaded_data_port
    exit()


def return_test_state_machine():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="first_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("MyFirstDataInputPort", "str")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    state2 = ExecutionState("MySecondState", path="../../test_scripts", filename="second_state.py")
    state2.add_outcome("FirstOutcome", 3)
    state2.add_input_data_port("DataInput1", "float")
    state2.add_output_data_port("DataOutput1", "float")

    state3 = HierarchyState("MyFirstHierarchyState", path="../../test_scripts", filename="hierarchy_container.py")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state2.state_id, 3, None, 6)
    state3.add_transition(state1.state_id, 3, state2.state_id, None)
    state3.add_input_data_port("in1", "str")
    state3.add_input_data_port("in2", "int")
    state3.add_output_data_port("out1", "str")
    state3.add_data_flow(state3.state_id,
                         state3.get_io_data_port_id_from_name_and_type("in1", DataPortType.INPUT),
                         state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("MyFirstDataInputPort", DataPortType.INPUT))
    state3.add_data_flow(state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("MyFirstDataOutputPort", DataPortType.OUTPUT),
                         state2.state_id,
                         state2.get_io_data_port_id_from_name_and_type("DataInput1", DataPortType.INPUT))
    state3.add_scoped_variable("scopeVar1", "str", "scopeDefaultValue")
    state3.add_scoped_variable("scopeVar2", "str", "scopeDefaultValue")
    return state3


def return_loop_state_machine():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="loop_state1.py")
    state1.add_outcome("MyFirstOutcome", 3)


    state2 = ExecutionState("MySecondState", path="../../test_scripts", filename="loop_state2.py")
    state2.add_outcome("FirstOutcome", 3)

    state3 = HierarchyState("MyFirstHierarchyState", path="../../test_scripts", filename="hierarchy_container.py")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.set_start_state(state1.state_id)
    state3.add_transition(state1.state_id, 3, state2.state_id, None)
    state3.add_transition(state2.state_id, 3, state1.state_id, None)

    return state3


def start_stop_pause_step_test():
    from mvc.models import ContainerStateModel
    from mvc.views.single_widget_window import TestButtonsView
    import gtk

    statemachine.singleton.state_machine_execution_engine.step_mode()

    s = Storage("../../test_scripts/stored_statemachine")
    state3 = return_loop_state_machine()
    s.save_statemachine_as_yaml(state3)
    root_state = s.load_statemachine_from_yaml()

    ctr_model = ContainerStateModel(root_state)
    test_buttons_view = TestButtonsView(ctr_model)

    root_state.daemon = True
    root_state.start()
    gtk.main()


# remember: scoped data is all data in a container state (including input_data, scoped variables and outputs of child
# states)
def scoped_data_test():
    s = Storage("../../test_scripts/stored_statemachine")
    state3 = return_test_state_machine()
    s.save_statemachine_as_yaml(state3)
    root_state = s.load_statemachine_from_yaml()
    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data
    root_state.start()
    root_state.join()


def default_data_port_values_test():
    s = Storage("../../test_scripts/stored_statemachine")

    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="first_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("MyFirstDataInputPort", "str", "default_value_test")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    state3 = HierarchyState("MyFirstHierarchyState", path="../../test_scripts", filename="hierarchy_container.py")
    state3.add_state(state1)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, None, 6)
    state3.add_input_data_port("in1", "str")
    state3.add_input_data_port("in2", "int")
    state3.add_output_data_port("out1", "str")

    s.save_statemachine_as_yaml(state3)
    root_state = s.load_statemachine_from_yaml()

    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    root_state.start()
    root_state.join()


def save_libraries():
    s = Storage("../")

    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="first_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    input_state1 = state1.add_input_data_port("MyFirstDataInputPort", "str")
    output_state1 = state1.add_output_data_port("MyFirstDataOutputPort", "float")

    state2 = ExecutionState("MySecondState", path="../../test_scripts", filename="second_state.py")
    state2.add_outcome("FirstOutcome", 3)
    input_state2 = state2.add_input_data_port("DataInput1", "float")
    output_state2 = state2.add_output_data_port("DataOutput1", "float")

    state3 = HierarchyState("Library1", path="../../test_scripts", filename="hierarchy_container.py")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state2.state_id, 3, None, 6)
    state3.add_transition(state1.state_id, 3, state2.state_id, None)
    input_state3 = state3.add_input_data_port("in1", "str")
    input2_state3 = state3.add_input_data_port("in2", "int")
    state3.add_output_data_port("out1", "str")
    state3.add_data_flow(state3.state_id,
                         input_state3,
                         state1.state_id,
                         input_state1)
    state3.add_data_flow(state1.state_id,
                         output_state1,
                         state2.state_id,
                         input_state2)

    s.save_statemachine_as_yaml(state3, "../../test_scripts/test_libraries/MyFirstLibrary", "0.1")
    state3.name = "Library2"
    s.save_statemachine_as_yaml(state3, "../../test_scripts/test_libraries/MySecondLibrary", "0.1")
    state3.name = "LibraryNested1"
    s.save_statemachine_as_yaml(state3, "../../test_scripts/test_libraries/LibraryContainer/Nested1", "0.1")
    state3.name = "LibraryNested2"
    s.save_statemachine_as_yaml(state3, "../../test_scripts/test_libraries/LibraryContainer/Nested2", "0.1")


def get_library_statemachine():
    statemachine.singleton.library_manager.initialize()
    library_container_state = HierarchyState("LibContainerState", path="../../test_scripts",
                                             filename="hierarchy_container.py")
    lib_state = LibraryState("test_libraries", "MyFirstLibrary", "0.1")
    library_container_state.add_state(lib_state)
    library_container_state.set_start_state(lib_state.state_id)
    library_container_state.add_outcome("Container_Outcome", 6)
    library_container_state.add_transition(lib_state.state_id, 6, None, 6)
    lib_container_input = library_container_state.add_input_data_port("in1", "str")
    library_container_state.add_output_data_port("out1", "str")
    library_container_state.add_data_flow(library_container_state.state_id,
                                          lib_container_input,
                                          lib_state.state_id,
                                          lib_state.get_io_data_port_id_from_name_and_type("in1", DataPortType.INPUT))
    return library_container_state


def run_library_statemachine():
    library_container_state = get_library_statemachine()
    input_data = {"in1": "input_string"}
    output_data = {"out1": None}
    library_container_state.input_data = input_data
    library_container_state.output_data = output_data
    library_container_state.start()
    library_container_state.join()


def save_nested_library_state():
    save_libraries()
    library_container_state = get_library_statemachine()
    statemachine.singleton.global_storage.save_statemachine_as_yaml(library_container_state,
                                "../../test_scripts/test_libraries/library_with_nested_library", "0.1")


def run_nested_library_statemachine():
    statemachine.singleton.library_manager.initialize()
    nested_lib_state = LibraryState("test_libraries", "library_with_nested_library", "0.1")
    input_data = {"in1": "input_string"}
    output_data = {"out1": None}
    nested_lib_state.input_data = input_data
    nested_lib_state.output_data = output_data
    nested_lib_state.start()
    nested_lib_state.join()

def scoped_variable_test():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="first_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("MyFirstDataInputPort", "str")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    state3 = HierarchyState("MyFirstHierarchyState", path="../../test_scripts", filename="hierarchy_container.py")
    state3.add_state(state1)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, None, 6)
    state3.add_input_data_port("in1", "str")
    state3.add_input_data_port("in2", "int")
    state3.add_output_data_port("out1", "str")
    state3.add_scoped_variable("scopeVar1", "str", "scopeDefaultValue")
    state3.add_data_flow(state3.state_id,
                         state3.get_scoped_variable_from_name("scopeVar1"),
                         state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("MyFirstDataInputPort", DataPortType.INPUT))

    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None}

    state3.input_data = input_data
    state3.output_data = output_data
    state3.start()
    state3.join()


def state_without_path_test():
    state1 = ExecutionState("MyFirstState")
    state1.add_outcome("Success", 0)
    state1.start()
    pass


if __name__ == '__main__':

    #start_stop_pause_step_test()

    #scoped_data_test()
    #save_and_load_data_port_test()
    #default_data_port_values_test()
    #scoped_variable_test()
    #hierarchy_test()
    #hierarchy_save_load_test()
    #state_without_path_test()

    #concurrency_barrier_test()
    #concurrency_barrier_save_load_test()
    #concurrency_preemption_test()
    #concurrency_preemption_save_load_test()

    #state_machine_manager_test()
    #external_modules_test()
    #global_variable_test()
    #ros_external_module_test()

    #save_libraries()
    #print "########################################################"
    # you have to run save_libraries() test before you can run run_library_statemachine()
    #run_library_statemachine()

    save_nested_library_state()
    #print "########################################################"
    # you have to run save_nested_library_state() test before you can run run_library_statemachine()
    run_nested_library_statemachine()

    #TODO: test
    # test data flow in barrier state machine
    # test data flow in preemptive state machine
    # test data flow between states consisting not of primitive data types
    # global variable stress tester

    #TODO: implement
    # write unit-tests
    # execution engine: especially pause and stop
    # execution history + step-mode

    #TODO: longterm
    # step back
    # validity checker
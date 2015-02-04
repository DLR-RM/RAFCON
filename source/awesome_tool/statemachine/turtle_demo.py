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


def run_turtle_demo():

    statemachine.singleton.state_machine_execution_engine.start()

    turtle_demo_state = PreemptiveConcurrencyState("TurtleDemo",
                                                   path="../../test_scripts/turtle_demo",
                                                   filename="turtle_demo.py")

    ########################################################
    # user controlled turtle substates
    ########################################################
    user_turtle_hierarchy_state = HierarchyState("UserControlledTurtleHierarchyState",
                                                 path="../../test_scripts/turtle_demo",
                                                 filename="user_controlled_turtle_hierarchy_state.py")
    user_turtle_hierarchy_state.add_outcome("Success", 0)

    user_turtle_state = ExecutionState("UserControlledTurtle", path="../../test_scripts/turtle_demo",
                                       filename="user_controlled_turtle.py")
    user_turtle_state.add_outcome("Success", 0)

    user_turtle_hierarchy_state.add_state(user_turtle_state)
    #user_turtle_state loops to itself
    user_turtle_hierarchy_state.add_transition(user_turtle_state.state_id, 0, user_turtle_state.state_id, None)
    user_turtle_hierarchy_state.set_start_state(user_turtle_state.state_id)

    turtle_demo_state.add_state(user_turtle_hierarchy_state)
    turtle_demo_state.add_outcome("Success", 0)
    turtle_demo_state.add_transition(user_turtle_hierarchy_state.state_id, 0, None, 0)

    ########################################################
    # follower turtle bot
    ########################################################


    ########################################################
    # food turtle bots
    ########################################################



    #statemachine.singleton.global_storage.save_statemachine_as_yaml(turtle_demo_state, "../../test_scripts/turtle_demo")
    #root_state = statemachine.singleton.global_storage.load_statemachine_from_yaml("../../test_scripts/turtle_demo")

    # input_data = {"MyFirstDataInputPort": "input_string"}
    # output_data = {"MyFirstDataOutputPort": None}
    # root_state.input_data = input_data
    # root_state.output_data = output_data

    ros_module = ExternalModule(name="ros", module_name="ros_external_module", class_name="RosModule")
    statemachine.singleton.external_module_manager.add_external_module(ros_module)
    statemachine.singleton.external_module_manager.external_modules["ros"].connect([])
    statemachine.singleton.external_module_manager.external_modules["ros"].start()

    user_input_module = ExternalModule(name="user_input", module_name="user_input_external_module", class_name="UserInput")
    statemachine.singleton.external_module_manager.add_external_module(user_input_module)
    statemachine.singleton.external_module_manager.external_modules["user_input"].connect([])

    turtle_demo_state.start()

    statemachine.singleton.external_module_manager.external_modules["user_input"].start()
    statemachine.singleton.external_module_manager.external_modules["user_input"].instance.gtk_worker_thread.join()

    turtle_demo_state.join()



if __name__ == '__main__':

    statemachine.singleton.state_machine_execution_engine.start()
    run_turtle_demo()

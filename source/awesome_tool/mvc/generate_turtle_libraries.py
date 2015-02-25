from statemachine.states.execution_state import ExecutionState
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.library_state import LibraryState
import statemachine.singleton
from statemachine.states.state import DataPortType
from statemachine.storage.storage import Storage
from statemachine.state_machine import StateMachine

storage = Storage("../test_scripts/test_libraries")


def generate_ros_init_node_library():
    init_ros_node = ExecutionState("init_ros_node", path="../../test_scripts/ros_libraries",
                                   filename="init_ros_node.py")
    init_ros_node.add_outcome("success", 0)

    node_name_input = init_ros_node.add_input_data_port("node_name", "str", "new_ros_node")


    storage.save_statemachine_as_yaml(init_ros_node,
                                      "../../test_scripts/ros_libraries/init_ros_node",
                                      "0.1",
                                      delete_old_state_machine=True)


def generate_spawn_turtle_library():
    spawn_turtle = ExecutionState("spawn_turtle", path="../../test_scripts/turtle_libraries", filename="spawn_turtle.py")
    spawn_turtle.add_outcome("success", 0)
    name_input = spawn_turtle.add_input_data_port("turtle_name", "str", "new_turtle")
    x_pos_input = spawn_turtle.add_input_data_port("x_pos", "float", 5.0)
    y_pos_input = spawn_turtle.add_input_data_port("y_pos", "float", 5.0)
    phi_input = spawn_turtle.add_input_data_port("phi", "float", 1.0)

    storage.save_statemachine_as_yaml(spawn_turtle,
                                      "../../test_scripts/turtle_libraries/spawn_turtle",
                                      "0.1",
                                      delete_old_state_machine=True)

def teleport_turtle_turtle_library():
    teleport_turtle = ExecutionState("spawn_turtle", path="../../test_scripts/turtle_libraries", filename="spawn_turtle.py")
    teleport_turtle.add_outcome("success", 0)
    name_input = teleport_turtle.add_input_data_port("turtle_name", "str", "new_turtle")
    x_pos_input = teleport_turtle.add_input_data_port("x_pos", "float", 5.0)
    y_pos_input = teleport_turtle.add_input_data_port("y_pos", "float", 5.0)
    phi_input = teleport_turtle.add_input_data_port("phi", "float", 1.0)

    storage.save_statemachine_as_yaml(spawn_turtle,
                                      "../../test_scripts/turtle_libraries/spawn_turtle",
                                      "0.1",
                                      delete_old_state_machine=True)

# TODO: teleport_turtle, clear_field, move_turtle, kill_turtle, turtle_position_subscriber

def generate_libraries():
    generate_ros_init_node_library()
    generate_spawn_turtle_library()


if __name__ == '__main__':
    generate_libraries()

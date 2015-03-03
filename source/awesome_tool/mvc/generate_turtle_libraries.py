from statemachine.states.execution_state import ExecutionState
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.library_state import LibraryState
import statemachine.singleton
from statemachine.states.state import DataPortType
from statemachine.storage.storage import Storage
from statemachine.state_machine import StateMachine

storage = Storage("../test_scripts/test_libraries")


def ros_init_node_library():
    init_ros_node = ExecutionState("init_ros_node", state_id="INIT_ROS_NODE", path="../../test_scripts/ros_libraries",
                                   filename="init_ros_node.py")
    init_ros_node.add_outcome("success", 0)

    node_name_input = init_ros_node.add_input_data_port("node_name", "str", "new_ros_node")

    storage.save_statemachine_as_yaml(init_ros_node,
                                      "../../test_scripts/ros_libraries/init_ros_node",
                                      "0.1",
                                      delete_old_state_machine=True)

def spawn_turtle_library():
    spawn_turtle = ExecutionState("spawn_turtle", state_id="SPAWN_TURTLE", path="../../test_scripts/turtle_libraries", filename="spawn_turtle.py")
    spawn_turtle.add_outcome("success", 0)
    name_input = spawn_turtle.add_input_data_port("turtle_name", "str", "new_turtle")
    x_pos_input = spawn_turtle.add_input_data_port("x_pos", "float", 5.0)
    y_pos_input = spawn_turtle.add_input_data_port("y_pos", "float", 5.0)
    phi_input = spawn_turtle.add_input_data_port("phi", "float", 1.0)

    storage.save_statemachine_as_yaml(spawn_turtle,
                                      "../../test_scripts/turtle_libraries/spawn_turtle",
                                      "0.1",
                                      delete_old_state_machine=True)


def teleport_turtle_library():
    teleport_turtle = ExecutionState("teleport_turtle", state_id="TELEPORT_TURTLE", path="../../test_scripts/turtle_libraries", filename="teleport_turtle.py")
    teleport_turtle.add_outcome("success", 0)
    name_input = teleport_turtle.add_input_data_port("turtle_name", "str", "new_turtle")
    x_pos_input = teleport_turtle.add_input_data_port("x_pos", "float", 1.0)
    y_pos_input = teleport_turtle.add_input_data_port("y_pos", "float", 1.0)
    phi_input = teleport_turtle.add_input_data_port("phi", "float", 1.0)

    storage.save_statemachine_as_yaml(teleport_turtle,
                                      "../../test_scripts/turtle_libraries/teleport_turtle",
                                      "0.1",
                                      delete_old_state_machine=True)


def clear_field_library():
    clear_field = ExecutionState("clear_field", state_id="CLEAR_FIELD", path="../../test_scripts/turtle_libraries", filename="clear_field.py")
    clear_field.add_outcome("success", 0)

    storage.save_statemachine_as_yaml(clear_field,
                                      "../../test_scripts/turtle_libraries/clear_field",
                                      "0.1",
                                      delete_old_state_machine=True)


def set_velocity_for_turtle_library():
    set_velocity = ExecutionState("move_turtle", state_id="SET_VELOCITY", path="../../test_scripts/turtle_libraries", filename="set_velocity.py")
    set_velocity.add_outcome("success", 0)
    name_input = set_velocity.add_input_data_port("turtle_name", "str", "new_turtle")
    x_vel_input = set_velocity.add_input_data_port("x_vel", "float", 1.0)
    phi_vel_input = set_velocity.add_input_data_port("phi_vel", "float", 0.3)

    storage.save_statemachine_as_yaml(set_velocity,
                                      "../../test_scripts/turtle_libraries/set_velocity",
                                      "0.1",
                                      delete_old_state_machine=True)


def kill_turtle_library():
    kill_turtle = ExecutionState("kill_turtle", state_id="KILL_TURTLE", path="../../test_scripts/turtle_libraries", filename="kill_turtle.py")
    kill_turtle.add_outcome("success", 0)
    name_input = kill_turtle.add_input_data_port("turtle_name", "str", "new_turtle")

    storage.save_statemachine_as_yaml(kill_turtle,
                                      "../../test_scripts/turtle_libraries/kill_turtle",
                                      "0.1",
                                      delete_old_state_machine=True)


def turtle_position_subscriber_library():
    turtle_position_subscriber = ExecutionState("turtle_position_subscriber",
                                                state_id="TURTLE_POSITION_SUBSCRIBER",
                                                path="../../test_scripts/turtle_libraries",
                                                filename="turtle_position_subscriber.py")
    turtle_position_subscriber.add_outcome("success", 0)
    name_input = turtle_position_subscriber.add_input_data_port("turtle_name", "str", "new_turtle")
    global_storage_id_input = turtle_position_subscriber.add_input_data_port("global_storage_id_of_turtle_pos", "str", "new_turtle")

    x_pos_output = turtle_position_subscriber.add_output_data_port("x_pos", "float")
    y_pos_output = turtle_position_subscriber.add_output_data_port("y_pos", "float")
    phi_output = turtle_position_subscriber.add_output_data_port("phi", "float")

    storage.save_statemachine_as_yaml(turtle_position_subscriber,
                                      "../../test_scripts/turtle_libraries/turtle_position_subscriber",
                                      "0.1",
                                      delete_old_state_machine=True)


def move_to_position_library():
    move_turtle = ExecutionState("move_turtle",
                                 state_id="MOVE_TO_POSITION",
                                 path="../../test_scripts/turtle_libraries",
                                 filename="move_to_position.py")
    move_turtle.add_outcome("success", 0)
    move_turtle.add_outcome("position_not_reached_yet", 1)
    global_storage_id_input = move_turtle.add_input_data_port("global_storage_id_of_turtle_pos", "str", "new_turtle")
    name_input = move_turtle.add_input_data_port("turtle_name", "str", "new_turtle")
    x_pos_input = move_turtle.add_input_data_port("x_pos", "float", 9.0)
    y_pos_input = move_turtle.add_input_data_port("y_pos", "float", 5.0)

    storage.save_statemachine_as_yaml(move_turtle,
                                      "../../test_scripts/turtle_libraries/move_to_position",
                                      "0.1",
                                      delete_old_state_machine=True)


def generate_libraries():
    ros_init_node_library()
    spawn_turtle_library()
    teleport_turtle_library()
    clear_field_library()
    set_velocity_for_turtle_library()
    kill_turtle_library()
    turtle_position_subscriber_library()
    move_to_position_library()


if __name__ == '__main__':
    generate_libraries()

from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.library_state import LibraryState
import rafcon.statemachine.singleton
from rafcon.statemachine.states.state import DataPortType
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine

storage = StateMachineStorage("../test_scripts/test_libraries")


#################################################################
# omnirob
#################################################################

def omnirob_move_base_rel():
    lib_node = ExecutionState("move_base_rel", state_id="OMNIROB_MOVE_BASE_REL", path="../../test_scripts/demo_libraries/omnirob",
                                   filename="move_base_abs.py")
    lib_node.add_outcome("success", 0)

    storage.save_statemachine_as_yaml(StateMachine(lib_node),
                                      "../../test_scripts/omnirob_libs/move_base_rel",
                                      "0.1",
                                      delete_old_state_machine=True)


def omnirob_move_base_abs():
    lib_node = ExecutionState("move_base_abs", state_id="OMNIROB_MOVE_BASE_ABS", path="../../test_scripts/demo_libraries/omnirob",
                                   filename="move_base_rel.py")
    lib_node.add_outcome("success", 0)

    storage.save_statemachine_as_yaml(StateMachine(lib_node),
                                      "../../test_scripts/omnirob_libs/move_base_abs",
                                      "0.1",
                                      delete_old_state_machine=True)


def omnirob_move_pan_tilt():
    lib_node = ExecutionState("move_pan_tilt", state_id="OMNIROB_MOVE_BASE_ABS", path="../../test_scripts/demo_libraries/omnirob",
                                   filename="move_pan_tilt.py")
    lib_node.add_outcome("success", 0)

    storage.save_statemachine_as_yaml(StateMachine(lib_node),
                                      "../../test_scripts/omnirob_libs/move_pan_tilt",
                                      "0.1",
                                      delete_old_state_machine=True)

#################################################################
# lwr
#################################################################

def lwr_move_along_joint_path():
    lib_node = ExecutionState("move_along_joint_path",
                              state_id="LWR_MOVE_ALONG_JOINT_PATH",
                              path="../../test_scripts/demo_libraries/lwr",
                              filename="move_along_joint_path.py")
    lib_node.add_outcome("success", 0)

    storage.save_statemachine_as_yaml(StateMachine(lib_node),
                                      "../../test_scripts/lwr_libs/move_along_joint_path",
                                      "0.1",
                                      delete_old_state_machine=True)


def lwr_move_single_joint():
    lib_node = ExecutionState("move_single_joint",
                              state_id="LWR_MOVE_SINGLE_JOINT",
                              path="../../test_scripts/demo_libraries/lwr",
                              filename="move_single_joint.py")
    lib_node.add_outcome("success", 0)

    storage.save_statemachine_as_yaml(StateMachine(lib_node),
                                      "../../test_scripts/lwr_libs/move_single_joint",
                                      "0.1",
                                      delete_old_state_machine=True)


def lwr_zero_gravity_mode():
    lib_node = ExecutionState("zero_gravity_mode",
                              state_id="LWR_ZERO_GRAVITY",
                              path="../../test_scripts/demo_libraries/lwr",
                              filename="zero_gravity_mode.py")
    lib_node.add_outcome("success", 0)

    storage.save_statemachine_as_yaml(StateMachine(lib_node),
                                      "../../test_scripts/lwr_libs/zero_gravity_mode",
                                      "0.1",
                                      delete_old_state_machine=True)


def lwr_move_to_cartesian_pose_impedance_controlled():
    lib_node = ExecutionState("move_to_cartesian_pose_impedance_controlled",
                              state_id="LWR_MOVE_TO_CARTESIAN_POSE_IMPEDANCE_CONTROLLED",
                              path="../../test_scripts/demo_libraries/lwr",
                              filename="move_to_cartesian_pose_impedance_controlled.py")
    lib_node.add_outcome("success", 0)

    storage.save_statemachine_as_yaml(StateMachine(lib_node),
                                      "../../test_scripts/lwr_libs/move_to_cartesian_pose_impedance_controlled",
                                      "0.1",
                                      delete_old_state_machine=True)


def lwr_move_to_cartesian_pose_position_controlled():
    lib_node = ExecutionState("move_to_cartesian_pose_position_controlled",
                              state_id="LWR_MOVE_TO_CARTESIAN_POSE_POSITION_CONTROLLED",
                              path="../../test_scripts/demo_libraries/lwr",
                              filename="move_to_cartesian_pose_position_controlled.py")
    lib_node.add_outcome("success", 0)

    storage.save_statemachine_as_yaml(StateMachine(lib_node),
                                      "../../test_scripts/lwr_libs/move_to_cartesian_pose_position_controlled",
                                      "0.1",
                                      delete_old_state_machine=True)
#################################################################
# lru
#################################################################


def generate_libraries():
    omnirob_move_base_rel()
    omnirob_move_base_abs()
    omnirob_move_pan_tilt()

    lwr_move_along_joint_path()
    lwr_move_single_joint()
    lwr_zero_gravity_mode()
    lwr_move_to_cartesian_pose_impedance_controlled()
    lwr_move_to_cartesian_pose_position_controlled()


if __name__ == '__main__':
    generate_libraries()

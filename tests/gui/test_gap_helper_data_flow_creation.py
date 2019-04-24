import pytest


def test_add_data_flow_to_state(mocker):
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.execution_state import ExecutionState

    from rafcon.gui.models.container_state import ContainerStateModel

    from rafcon.gui.mygaphas.utils.gap_helper import add_data_flow_to_state

    root_state = HierarchyState("Root", "root")
    child_container = HierarchyState("ChildContainer", "cc")
    child_execution = ExecutionState("ChildExecution", "ce")
    root_state.add_state(child_container)
    root_state.add_state(child_execution)

    floating_state = HierarchyState("Floater", "f")

    root_state.add_input_data_port("ri1", int, data_port_id=0)
    root_state.add_input_data_port("ri2", int, data_port_id=1)
    root_state.add_output_data_port("ro1", int, data_port_id=2)
    root_state.add_scoped_variable("rs1", int, scoped_variable_id=3)

    child_container.add_input_data_port("ci1", int, data_port_id=0)
    child_container.add_output_data_port("co1", int, data_port_id=2)

    child_execution.add_input_data_port("ei1", int, data_port_id=0)
    child_execution.add_output_data_port("eo1", int, data_port_id=2)

    floating_state.add_input_data_port("fi1", int, data_port_id=0)
    floating_state.add_output_data_port("fo1", int, data_port_id=1)

    root_state_m = ContainerStateModel(root_state)
    child_container_m = root_state_m.states["cc"]
    child_execution_m = root_state_m.states["ce"]

    floating_state_m = ContainerStateModel(floating_state)

    ri1_m = root_state_m.get_input_data_port_m(0)
    ro1_m = root_state_m.get_output_data_port_m(2)
    rs1_m = root_state_m.get_scoped_variable_m(3)

    ci1_m = child_container_m.get_input_data_port_m(0)
    co1_m = child_container_m.get_output_data_port_m(2)

    ei1_m = child_execution_m.get_input_data_port_m(0)
    eo1_m = child_execution_m.get_output_data_port_m(2)

    fi1_m = floating_state_m.get_input_data_port_m(0)
    fo1_m = floating_state_m.get_output_data_port_m(1)

    with mocker.patch.object(root_state, "add_data_flow"), mocker.patch.object(child_container, "add_data_flow"):
        # Data flow in root state from input to output
        add_data_flow_to_state(ri1_m, ro1_m)
        root_state.add_data_flow.assert_called_with("root", 0, "root", 2)
        # Data flow in root state from input to scope
        add_data_flow_to_state(ri1_m, rs1_m)
        root_state.add_data_flow.assert_called_with("root", 0, "root", 3)
        # Data flow in root state from scope to output
        add_data_flow_to_state(rs1_m, ro1_m)
        root_state.add_data_flow.assert_called_with("root", 3, "root", 2)

        # Data flow from root state input to container state input
        add_data_flow_to_state(ri1_m, ci1_m)
        root_state.add_data_flow.assert_called_with("root", 0, "cc", 0)
        # Data flow from root state input to execution state input
        add_data_flow_to_state(ri1_m, ei1_m)
        root_state.add_data_flow.assert_called_with("root", 0, "ce", 0)

        # Data flow from container state output to execution state input
        add_data_flow_to_state(co1_m, ei1_m)
        root_state.add_data_flow.assert_called_with("cc", 2, "ce", 0)
        # Data flow from execution state output to container state input
        add_data_flow_to_state(eo1_m, ci1_m)
        root_state.add_data_flow.assert_called_with("ce", 2, "cc", 0)

        # Data flow from container state output to container state input, see issue #711
        add_data_flow_to_state(co1_m, ci1_m)
        root_state.add_data_flow.assert_called_with("cc", 2, "cc", 0)
        # Data flow from execution state output to container state input, see issue #711
        add_data_flow_to_state(eo1_m, ei1_m)
        root_state.add_data_flow.assert_called_with("ce", 2, "ce", 0)

        # Data flow from container state output to root state output
        add_data_flow_to_state(co1_m, ro1_m)
        root_state.add_data_flow.assert_called_with("cc", 2, "root", 2)
        # Data flow from execution state output to root state output
        add_data_flow_to_state(eo1_m, ro1_m)
        root_state.add_data_flow.assert_called_with("ce", 2, "root", 2)
        # Data flow from execution state output to root state scope
        add_data_flow_to_state(eo1_m, rs1_m)
        root_state.add_data_flow.assert_called_with("ce", 2, "root", 3)

        # Data flow from floating state output to execution state input
        with pytest.raises(ValueError):
            add_data_flow_to_state(fo1_m, ei1_m)
        # Data flow from execution state output to floating state input
        with pytest.raises(ValueError):
            add_data_flow_to_state(eo1_m, fi1_m)



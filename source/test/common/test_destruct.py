import os
import pytest

import rafcon
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState

import test_basic_state_machine as basic_state_machines
import testing_utils

FILES = [RAFCON_TEMP_PATH_BASE + '/state_generation_log_file.txt',
         RAFCON_TEMP_PATH_BASE + '/state_del_log_file.txt',
         RAFCON_TEMP_PATH_BASE + '/state_element_generation_log_file.txt',
         RAFCON_TEMP_PATH_BASE + '/state_element_del_log_file.txt']

old_state_init = rafcon.statemachine.states.state.State.__init__
old_state_del = None
if hasattr(rafcon.statemachine.states.state.State, '__del__'):
    old_state_del = rafcon.statemachine.states.state.State.__del__
old_state_element_init = rafcon.statemachine.state_elements.state_element.StateElement.__init__
old_state_element_del = None
if hasattr(rafcon.statemachine.state_elements.state_element.StateElement, '__del__'):
        old_state_element_del = rafcon.statemachine.state_elements.state_element.StateElement.__del__


def create_models(*args, **kargs):

    state1 = ExecutionState('State1', state_id='STATE1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2', state_id='STATE2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = HierarchyState(name='Nested', state_id='NESTED')
    state4.add_outcome('GoGo')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2', state_id='NESTED2')
    state5.add_outcome('HereWeGo')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3', state_id='STATE3')
    input_state3 = state3.add_input_data_port("input", "int", 0)
    output_state3 = state3.add_output_data_port("output", "int")
    state3.add_state(state4)
    state3.add_state(state5)
    state3.set_start_state(state4)
    state3.add_scoped_variable("share", "int", 3)
    state3.add_transition(state4.state_id, 0, state5.state_id, None)
    state3.add_transition(state5.state_id, 0, state3.state_id, 0)
    state3.add_data_flow(state4.state_id, output_state4, state5.state_id, input_state5)
    state3.add_outcome('Branch1')
    state3.add_outcome('Branch2')

    ctr_state = HierarchyState(name="Container", state_id='CONT2')
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    input_ctr_state = ctr_state.add_input_data_port("ctr_in", "str", "zero")
    output_ctr_state = ctr_state.add_output_data_port("ctr_out", "int")
    ctr_state.set_start_state(state1)
    ctr_state.add_transition(state1.state_id, 0, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, 0, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, 0, ctr_state.state_id, 0)
    ctr_state.add_data_flow(state1.state_id, output_state1, state2.state_id, input_par_state2)
    ctr_state.add_data_flow(state2.state_id, output_res_state2, state3.state_id, input_state3)
    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, state1.state_id, input_state1)
    ctr_state.add_data_flow(state3.state_id, output_state3, ctr_state.state_id, output_ctr_state)
    ctr_state.name = "Container"

    ctr_state.add_input_data_port("input", "str", "default_value1")
    ctr_state.add_input_data_port("pos_x", "str", "default_value2")
    ctr_state.add_input_data_port("pos_y", "str", "default_value3")

    ctr_state.add_output_data_port("output", "str", "default_value1")
    ctr_state.add_output_data_port("result", "str", "default_value2")

    scoped_variable1_ctr_state = ctr_state.add_scoped_variable("scoped", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("ctr", "int", 42)

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    state_dict = {'Container': ctr_state, 'State1': state1, 'State2': state2, 'State3': state3, 'Nested': state4,
                  'Nested2': state5}

    return ctr_state, state_dict


def get_log_elements(with_prints=False):
    state_del_file = []
    for line in open(RAFCON_TEMP_PATH_BASE + '/state_del_log_file.txt', 'a+'):
        state_del_file.append(line)
    state_element_del_file = []
    for line in open(RAFCON_TEMP_PATH_BASE + '/state_element_del_log_file.txt', 'a+'):
        state_element_del_file.append(line)
    state_gen_file = []
    for line in open(RAFCON_TEMP_PATH_BASE + '/state_generation_log_file.txt', 'a+'):
        state_gen_file.append(line)
    state_element_gen_file = []
    for line in open(RAFCON_TEMP_PATH_BASE + '/state_element_generation_log_file.txt', 'a+'):
        state_element_gen_file.append(line)

    for elem in state_gen_file:
        name_and_id = elem.split("'")
        mem_id = elem.split("]")
        act_elem = name_and_id[1] + name_and_id[3] + mem_id[-1]
        # if str(mem_id[-1]) not in [str(e.split("]")[-1]) for e in state_del_file]:
        if with_prints:
            if act_elem not in [e.split("'")[1] + e.split("'")[3] + e.split("]")[-1] for e in state_del_file]:
                print "State object still in memory: \n", elem, "\n", ''.join(state_del_file)
            # else:
            #     print "State object destroyed: \n", elem, '\n', ''.join(state_del_file)

    for elem in state_element_gen_file:
        mem_id = elem.split(' ')[-1]
        # if elem not in state_element_del_file:
        if with_prints:
            if mem_id not in [e.split(' ')[-1] for e in state_element_del_file]:
                print "State-Element object still in memory: \n", elem, '\n', ''.join(state_element_del_file)
            # else:
            #     print "State-Element object destroyed: \n", elem, '\n', ''.join(state_element_del_file)
    print '\n'.join([log_file_path for log_file_path in FILES])

    return state_gen_file, state_del_file, state_element_gen_file, state_element_del_file


def remove_log_files():
    print "REMOVE: \n{}".format('\n'.join([log_file_path for log_file_path in FILES]))
    for log_file_path in FILES:
        if os.path.exists(log_file_path):
            os.remove(log_file_path)


def patch_core_classes_with_log():

    def state_init(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                   parent=None):
        self._patch = None
        self._name = name
        if state_id is None:
            self._state_id = rafcon.statemachine.states.state.state_id_generator()
        else:
            self._state_id = state_id
        with open(RAFCON_TEMP_PATH_BASE + '/state_generation_log_file.txt', 'a+') as f:
            f.write("RUN STATE of {0} {1}\n".format(self, id(self)))
        old_state_init(self, name, self._state_id, input_data_ports, output_data_ports, outcomes, parent)

    def state_element_init(self, parent=None):
        self._patch = None
        with open(RAFCON_TEMP_PATH_BASE + '/state_element_generation_log_file.txt', 'a+') as f:
            f.write("RUN STATE-ELEMENT of {0} {1}\n".format(self, id(self)))
        old_state_element_init(self, parent)

    def state_del(self):
        if old_state_del is not None:
            old_state_del(self)
        if hasattr(self, '_patch'):
            with open(RAFCON_TEMP_PATH_BASE + '/state_del_log_file.txt', 'a+') as f:
                f.write("RUN STATE of {0} {1}\n".format(self, id(self)))

    def state_element_del(self):
        if old_state_element_del is not None:
            old_state_element_del(self)
        if hasattr(self, '_patch'):
            with open(RAFCON_TEMP_PATH_BASE + '/state_element_del_log_file.txt', 'a+') as f:
                f.write("RUN STATE-ELEMENT of {0} {1}\n".format(self, id(self)))

    rafcon.statemachine.states.state.State.__init__ = state_init
    rafcon.statemachine.states.state.State.__del__ = state_del
    rafcon.statemachine.state_elements.state_element.StateElement.__init__ = state_element_init
    rafcon.statemachine.state_elements.state_element.StateElement.__del__ = state_element_del


def un_patch_core_classes_from_log():
    rafcon.statemachine.states.state.State.__init__ = old_state_init
    rafcon.statemachine.states.state.State.__del__ = old_state_del
    rafcon.statemachine.state_elements.state_element.StateElement.__init__ = old_state_element_init
    rafcon.statemachine.state_elements.state_element.StateElement.__del__ = old_state_element_del


def test_core_destruct(caplog):

    testing_utils.test_multithrading_lock.acquire()
    remove_log_files()

    patch_core_classes_with_log()

    basic_state_machines.test_create_state(caplog)

    basic_state_machines.test_create_container_state(caplog)

    basic_state_machines.test_port_and_outcome_removal(caplog)

    state_gen_file, state_del_file, state_element_gen_file, state_element_del_file = get_log_elements(with_prints=True)

    ratio = float(len(state_del_file))/float(len(state_gen_file))
    diff = len(state_gen_file) - len(state_del_file)
    print "Ratio of destroyed/generated state objects is: ", ratio, diff
    assert 0 == diff
    ratio = float(len(state_element_del_file))/float(len(state_element_gen_file))
    diff = len(state_element_gen_file) - len(state_element_del_file)
    print "Ratio of destroyed/generated state-elements objects is: ", ratio, diff
    assert 0 == diff
    un_patch_core_classes_from_log()
    remove_log_files()
    testing_utils.test_multithrading_lock.release()


def _test_model_and_core_destruct(caplog):
    # TODO test full model and core destruction if state machine is removed
    # -> perform test in a separate branch at the moment
    pass


def _test_model_and_core_destruct_with_gui(caplog):
    # TODO test full model and core destruction if state machine is removed and all gui widget active
    # -> perform test in a separate branch at the moment
    pass


if __name__ == '__main__':
    # test_core_destruct(None)
    pytest.main(['-s', __file__])

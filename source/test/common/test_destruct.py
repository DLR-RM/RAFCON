import os
import pytest

import rafcon
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE

import test_states as basic_state_machines
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


def get_log_elements(with_prints=False):

    with open(RAFCON_TEMP_PATH_BASE + '/state_del_log_file.txt') as f:
        state_del_file = f.readlines()
    with open(RAFCON_TEMP_PATH_BASE + '/state_element_del_log_file.txt') as f:
        state_element_del_file = f.readlines()
    with open(RAFCON_TEMP_PATH_BASE + '/state_generation_log_file.txt') as f:
        state_gen_file = f.readlines()
    with open(RAFCON_TEMP_PATH_BASE + '/state_element_generation_log_file.txt') as f:
        state_element_gen_file = f.readlines()

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

    # Ensure that the garbage collector has removed all unreferenced objects
    import gc
    gc.collect()

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

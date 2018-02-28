import os
from os.path import join
import copy
import time

import testing_utils
import pytest

with_print = False


def create_models():
    import rafcon.core.singleton
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.state_machine import StateMachine

    state1 = ExecutionState('State1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = HierarchyState(name='Nested')
    state4.add_outcome('GoGo')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2')
    state5.add_outcome('HereWeGo')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3')
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

    ctr_state = HierarchyState(name="Container")
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

    ctr_state.add_input_data_port("input", "str", "default_value1")
    ctr_state.add_input_data_port("pos_x", "str", "default_value2")
    ctr_state.add_input_data_port("pos_y", "str", "default_value3")

    ctr_state.add_output_data_port("output", "str", "default_value1")
    ctr_state.add_output_data_port("result", "str", "default_value2")

    scoped_variable1_ctr_state = ctr_state.add_scoped_variable("scoped", "str", "default_value1")
    scoped_variable2_ctr_state = ctr_state.add_scoped_variable("my_var", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("ctr", "int", 42)

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    state_dict = {'Container': ctr_state, 'State1': state1, 'State2': state2, 'State3': state3, 'Nested': state4, 'Nested2': state5}
    sm = StateMachine(ctr_state)
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)

    testing_utils.wait_for_gui()

    rafcon.core.singleton.state_machine_manager.active_state_machine_id = sm.state_machine_id

    testing_utils.wait_for_gui()

    state_machine_model = rafcon.gui.singleton.state_machine_manager_model.state_machines[sm.state_machine_id]

    return ctr_state, state_machine_model, state_dict


def create_models_lib(output_list):
    from rafcon.core.states.library_state import LibraryState

    [state, sm_model, state_dict] = create_models()

    wait3 = LibraryState(name="Wait3", library_path="generic", library_name="wait")
    state_dict['Nested'].add_state(wait3)
    dialog2 = LibraryState(name="2 Option", library_path=join("generic", "dialog"), library_name="Dialog [2 options]")
    dialog3 = LibraryState(name="3 Option", library_path=join("generic", "dialog"), library_name="Dialog [3 options]")
    state_dict['Nested'].add_state(dialog2)
    state_dict['Container'].add_state(dialog3)
    last_wins = LibraryState(name="last wins", library_path="unit_test_state_machines", library_name="last_data_wins_test")
    new_state_id = state_dict['Container'].add_state(last_wins.state_copy)

    for state_id in sm_model.root_state.states[new_state_id].states:
        state_m = sm_model.root_state.states[new_state_id].states[state_id]
        print state_m.state.state_id, state_m.state.get_path(), state_m.meta

    # sm_loaded = storage.load_state_machine_from_path(
    #     os.path.join(testing_utils.TEST_PATH, "assets", "unit_test_state_machines", "last_data_wins_test"))
    # # root_state = sm_loaded.root_state
    # # state_machine = StateMachine(root_state)
    # rafcon.core.singleton.state_machine_manager.add_state_machine(sm_loaded)
    # sm_model = rafcon.gui.singleton.state_machine_manager_model.state_machines[sm_loaded.state_machine_id]
    # return sm_loaded.root_state, sm_model, {}
    output_list.append(sm_model)


def create_models_concurrency():
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
    from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
    from rafcon.core.states.hierarchy_state import HierarchyState

    [state, sm_model, state_dict] = create_models()

    pstate = PreemptiveConcurrencyState(name='Preemptive', state_id='PREEMPT')
    state_dict['Nested'].add_state(pstate)
    state_dict['Preemptive'] = pstate
    hstate = HierarchyState()
    estate = ExecutionState()
    estate.script_text = estate.script_text + "\nprint 'nochwas'"
    state_dict['Preemptive'].add_state(hstate)
    state_dict['Preemptive'].add_state(estate)
    bstate = BarrierConcurrencyState(name='Barrier', state_id='BARRIER')
    state_dict['Nested'].add_state(bstate)
    state_dict['Barrier'] = bstate
    hstate = HierarchyState()
    estate = ExecutionState()
    estate.script_text = estate.script_text + "\nprint 'irgendwas'"
    state_dict['Barrier'].add_state(hstate)
    state_dict['Barrier'].add_state(estate)

    return state, sm_model, state_dict


def collect_state_memory_addresses(state, address_book=None):
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.container_state import ContainerState
    from rafcon.core.states.library_state import LibraryState

    # print type(state)
    if address_book is None:
        address_book = []

    def append_addresses_of_dict_elements(dict_of_elements, address_book):
        for elem in dict_of_elements.itervalues():
            # print "{0}: {1}".format(elem.__class__.__name__, id(elem))
            address_book.append(id(elem))

    if isinstance(state, LibraryState):
        # print "state_copy: {}".format(id(state.state_copy))
        collect_state_memory_addresses(state.state_copy, address_book)
        address_book.append(id(state.state_copy))
    else:
        # collect all memory addresses for, io-ports, outcomes and description
        append_addresses_of_dict_elements(state.input_data_ports, address_book)
        append_addresses_of_dict_elements(state.output_data_ports, address_book)
        append_addresses_of_dict_elements(state.outcomes, address_book)
        # print "description: {}".format(id(state.description))
        # address_book.append(id(state.description))

    if isinstance(state, ContainerState):
        append_addresses_of_dict_elements(state.data_flows, address_book)
        append_addresses_of_dict_elements(state.transitions, address_book)
        append_addresses_of_dict_elements(state.scoped_variables, address_book)
        for elem in state.states.itervalues():
            # print "{0}: {1}".format(elem.__class__.__name__, id(elem))
            collect_state_memory_addresses(elem, address_book)
            address_book.append(id(elem))

    if isinstance(state, ExecutionState):
        # print "script: {}".format(id(state.script))
        address_book.append(id(state.script))
        # print "script_text: {}".format(id(state.script_text))
        # address_book.append(id(state.script_text))

    return address_book


def collect_state_model_memory_addresses(state_m, address_book=None):
    from rafcon.core.states.container_state import ContainerState

    # print type(state_m)
    if address_book is None:
        address_book = []

    def append_addresses_of_list_elements(list_of_elements, address_book):
        for elem in list_of_elements:
            # print "{3} {0}: {1} {2}".format(elem.__class__.__name__, id(elem), id(elem.meta), state_m.state.get_path())
            address_book.append(id(elem))
            address_book.append(id(elem.meta))

    # collect all memory addresses for, io-ports, outcomes and description
    append_addresses_of_list_elements(state_m.input_data_ports, address_book)
    append_addresses_of_list_elements(state_m.output_data_ports, address_book)
    append_addresses_of_list_elements(state_m.outcomes, address_book)

    if isinstance(state_m.state, ContainerState):
        append_addresses_of_list_elements(state_m.data_flows, address_book)
        append_addresses_of_list_elements(state_m.transitions, address_book)
        append_addresses_of_list_elements(state_m.scoped_variables, address_book)
        for elem in state_m.states.itervalues():
            # print "{3} {0}: {1} {2}".format(elem.__class__.__name__, id(elem), id(elem.meta), state_m.state.get_path())
            address_book.append(id(elem))
            address_book.append(id(elem.meta))
            collect_state_model_memory_addresses(elem, address_book)

    return address_book


def compare_references_to_sm_model_and_core(sm_m, new_sm_m):
    sm = sm_m.state_machine
    new_sm = new_sm_m.state_machine
    old_address_book = collect_state_memory_addresses(sm.root_state)
    new_address_book = collect_state_memory_addresses(new_sm.root_state)
    merge_address_books = copy.copy(old_address_book)
    merge_address_books.extend(new_address_book)
    for address in old_address_book:
        if address in new_address_book:
            print address
    print 'C {0} + {1} = {2} == {3}'.format(len(old_address_book), len(new_address_book), len(set(merge_address_books)),
                                            len(old_address_book) + len(new_address_book))
    assert len(old_address_book) + len(new_address_book) == len(set(merge_address_books))
    old_address_book = collect_state_model_memory_addresses(sm_m.root_state)
    new_address_book = collect_state_model_memory_addresses(new_sm_m.root_state)
    merge_address_books = copy.copy(old_address_book)
    merge_address_books.extend(new_address_book)
    for address in old_address_book:
        if address in new_address_book:
            print address
    print 'M {0} + {1} = {2} == {3}'.format(len(old_address_book), len(new_address_book), len(set(merge_address_books)),
                                            len(old_address_book) + len(new_address_book))
    assert len(old_address_book) + len(new_address_book) == len(set(merge_address_books))


def equal_check_state(origin_state, target_state):
    print 'EQUAL STATE TEST '
    assert origin_state == target_state


def equal_check_state_model(origin_state_m, target_state_m):
    print 'EQUAL STATE MODEL TEST'
    assert origin_state_m == target_state_m
    # # print type(state_m)
    # def diff_of_list_elements(origin_list_of_elements, target_list_of_elements, name):
    #     origin_dict = {}
    #     id_name = name + '_id' if 'scope' not in name else 'data_port_id'
    #     for elem in origin_list_of_elements:
    #         origin_dict[getattr(getattr(elem, name), id_name)] = elem
    #     for elem in target_list_of_elements:
    #         # print "{0}: {1} {2}".format(elem.__class__.__name__, elem.meta, origin_meta[getattr(getattr(elem, name), id_name)])
    #         assert str(elem.meta) == str(origin_dict[getattr(getattr(elem, name), id_name)].meta)
    #         # assert elem == origin_dict[getattr(getattr(elem, name), id_name)]
    #         del origin_dict[getattr(getattr(elem, name), id_name)]
    #
    # # check all meta of state
    # # print "{3} {0}: {1} {2}".format(origin_state_m.__class__.__name__, origin_state_m.meta, target_state_m.meta, origin_state_m.state.get_path())
    # assert str(origin_state_m.meta) == str(target_state_m.meta)
    #
    # # check all meta for, io-ports, outcomes
    # diff_of_list_elements(origin_state_m.input_data_ports, target_state_m.input_data_ports, 'data_port')
    # diff_of_list_elements(origin_state_m.output_data_ports, target_state_m.output_data_ports, 'data_port')
    # diff_of_list_elements(origin_state_m.outcomes, target_state_m.outcomes, 'outcome')
    #
    # if isinstance(origin_state_m.state, ContainerState):
    #     diff_of_list_elements(origin_state_m.data_flows, target_state_m.data_flows, 'data_flow')
    #     diff_of_list_elements(origin_state_m.transitions, target_state_m.transitions, 'transition')
    #     diff_of_list_elements(origin_state_m.scoped_variables, target_state_m.scoped_variables, 'scoped_variable')
    #     for elem_id, elem in origin_state_m.states.iteritems():
    #         # if elem_id not in target_state_m.states:
    #         #     print "XXX ", target_state_m.state.get_path(), target_state_m.state.name, elem_id
    #         #     print "\n".join([str((elem, elem_id)) for elem, elem_id in origin_state_m.states.iteritems()])
    #         #     print "XXX ", elem.state.get_path(), elem.state.name
    #         equal_check_state_model(elem, target_state_m.states[elem_id])


def run_copy_test(sm_m, with_gui=False):
    """Run general test that """
    import rafcon.gui.singleton
    sm = sm_m.state_machine
    new_sm_m = copy.copy(sm_m)
    equal_check_state(sm_m.root_state.state, new_sm_m.root_state.state)
    equal_check_state_model(sm_m.root_state, new_sm_m.root_state)
    compare_references_to_sm_model_and_core(sm_m, new_sm_m)

    # storage copy tests
    # RAFCON_TEMP_PATH_BASE = "/net/notos/home_local/dark_room"
    # RAFCON_TEMP_PATH_BASE = "/volume/USERSTORE/beld_rc/tmp"
    if sm.file_system_path is None:
        tmp_sm_system_path = join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, 'copy_test_' + str(sm.state_machine_id))
    else:
        tmp_sm_system_path = join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, 'copy_test' + sm.file_system_path)

    new_sm_m.state_machine.root_state.name = "Copied Cont state"
    testing_utils.wait_for_gui()
    new_sm_m.destroy()
    if with_gui:
        main_window_controller = rafcon.gui.singleton.main_window_controller
        menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
        testing_utils.call_gui_callback(sm_m.state_machine.__setattr__, "file_system_path", tmp_sm_system_path)
        testing_utils.call_gui_callback(menubar_ctrl.on_save_activate, None)


def run_copy_performance_test_and_check_storage_copy(*args):
    """Run general test that """
    from rafcon.gui.models.state_machine import StateMachineModel
    from rafcon.core.storage import storage

    sm_m = args[0]
    sm = sm_m.state_machine

    # storage copy tests
    # RAFCON_TEMP_PATH_BASE = "/net/notos/home_local/dark_room"
    # RAFCON_TEMP_PATH_BASE = "/volume/USERSTORE/beld_rc/tmp"
    if sm.file_system_path is None:
        tmp_sm_system_path = join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, 'copy_test_' + str(sm.state_machine_id))
    else:
        tmp_sm_system_path = join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, 'copy_test' + sm.file_system_path)

    # performance tests
    time_only_storage_before = time.time()
    storage.save_state_machine_to_path(sm_m.state_machine, tmp_sm_system_path, delete_old_state_machine=False,
                                       as_copy=True)
    sm_m.store_meta_data(copy_path=tmp_sm_system_path)
    time_only_storage_after = time.time()
    only_storage_duration = round(time_only_storage_after*1000000) - round(time_only_storage_before*1000000)

    time_storage_before = time.time()
    storage.save_state_machine_to_path(sm_m.state_machine, tmp_sm_system_path, delete_old_state_machine=False,
                                       as_copy=True)
    sm_m.store_meta_data(copy_path=tmp_sm_system_path)

    sm1 = storage.load_state_machine_from_path(tmp_sm_system_path)
    time_model_before = time.time()
    sm1_m = StateMachineModel(sm1)
    time_model_after = time.time()
    time_storage_after = time_model_after
    only_model_duration = round(time_model_after*1000000) - round(time_model_before*1000000)
    storage_copy_duration = round(time_storage_after*1000000) - round(time_storage_before*1000000)
    equal_check_state(sm_m.root_state.state, sm1_m.root_state.state)
    equal_check_state_model(sm_m.root_state, sm1_m.root_state)

    time_copy_before = time.time()
    copy.copy(sm_m.state_machine)
    time_copy_after = time.time()
    core_copy_duration = round(time_copy_after*1000000) - round(time_copy_before*1000000)

    time_copy_m_before = time.time()
    copy.copy(sm_m)
    time_copy_m_after = time.time()
    model_copy_duration = round(time_copy_m_after*1000000) - round(time_copy_m_before*1000000)

    sm1_m.destroy()

    print "only_model_duration: {}".format(only_model_duration)
    print "only_storage_duration: {}".format(only_storage_duration)
    print "storage_copy_duration: {}".format(storage_copy_duration)
    print "core_copy_duration: {}".format(core_copy_duration)
    print "model_copy_duration: {}".format(model_copy_duration)


def test_simple(caplog):
    """Do all copy strategies possible in RAFCON and check if all Objects have different memory location to secure
    reference free assignments from origin to new state.
    :param caplog:
    :return:
    """
    testing_utils.dummy_gui(None)
    print "start test simple"
    # create testbed
    testing_utils.initialize_environment(gui_already_started=False,
                                         gui_config={'HISTORY_ENABLED': False,
                                                     'AUTO_BACKUP_ENABLED': False},
                                         )

    [state, sm_model, state_dict] = create_models()
    run_copy_test(sm_model)
    run_copy_performance_test_and_check_storage_copy(sm_model)
    # currently destroy doesn't do anything if auto_backup is disabled
    sm_model.destroy()
    import rafcon
    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()

    [state, sm_model, state_dict] = create_models_concurrency()
    run_copy_test(sm_model)
    run_copy_performance_test_and_check_storage_copy(sm_model)
    # currently destroy doesn't do anything if auto_backup is disabled
    sm_model.destroy()
    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
    # wait until state machine model is destroyed
    testing_utils.wait_for_gui()
    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)
    print "test simple finished"


def test_complex(caplog):
    """Do all copy strategies possible in RAFCON and check if all Objects have different memory location to secure
    reference free assignments from origin to new state.
    :param caplog:
    :return:
    """
    testing_utils.dummy_gui(None)
    with_gui = True

    if with_gui:
        print "test_complex with gui"
        try:
            testing_utils.run_gui(
                gui_config={'HISTORY_ENABLED': False,
                            'AUTO_BACKUP_ENABLED': False},
                libraries={"unit_test_state_machines":
                               os.path.join(testing_utils.TEST_ASSETS_PATH, "unit_test_state_machines")}
            )
            output_list = list()
            testing_utils.call_gui_callback(create_models_lib, output_list)
            sm_model = output_list[0]
            testing_utils.call_gui_callback(run_copy_test, sm_model, with_gui=True)
            testing_utils.call_gui_callback(run_copy_performance_test_and_check_storage_copy, sm_model)
        except:
            raise
        finally:
            testing_utils.close_gui()
            testing_utils.shutdown_environment(caplog=caplog)
        print "finish test_complex with gui"

        # import threading
        # print "&" * 50
        # print "end of copy method"
        # print threading.currentThread().ident
        # print threading.currentThread()
        # print "&" * 50
    else:
        print "test_complex without gui"
        testing_utils.initialize_environment(
            gui_config={'HISTORY_ENABLED': False,
                        'AUTO_BACKUP_ENABLED': False},
            libraries={"unit_test_state_machines":
                           os.path.join(testing_utils.TEST_ASSETS_PATH, "unit_test_state_machines")},
            gui_already_started=False)

        output_list = list()
        create_models_lib(output_list)
        sm_model = output_list[0]
        run_copy_test(sm_model)
        run_copy_performance_test_and_check_storage_copy(sm_model)
        sm_model.destroy()
        import rafcon.core.singleton
        rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)
        print "after test_complex without gui"

    # import conftest
    # import shutil
    # for elem in os.listdir(testing_utils.constants.RAFCON_TEMP_PATH_BASE):
    #     path = os.path.join(testing_utils.constants.RAFCON_TEMP_PATH_BASE, elem)
    #     if os.path.isdir(path) and not path == testing_utils.RAFCON_TEMP_PATH_TEST_BASE:
    #         shutil.rmtree(path)

    # This test must not be called by py.test directly!
    # As it is a test without gui it must not create the core and gui singletons,
    # otherwise the multi-threading test will fail
    # test_simple(caplog)


if __name__ == '__main__':
    testing_utils.dummy_gui(None)
    # import cProfile
    # import re
    # import copy
    # cProfile.run('test_state_add_remove_notification(None)')
    test_complex(None)
    # test_simple(None)
    # pytest.main(['-s', __file__])

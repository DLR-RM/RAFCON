import os
import gc
import time
import shutil
import gtkmvc
from pprint import pprint

import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
import rafcon.core.states.state
import rafcon.core.state_elements.state_element

from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE

import testing_utils
from rafcon.utils import log
logger = log.get_logger(__name__)

GENERATION_LOG_FILE_APPENDIX = 'gen_log_file.txt'
DELETION_LOG_FILE_APPENDIX = 'del_log_file.txt'

# store method of core element classes
CORE_FILES = ['state', 'state_element']
old_state_init = None
old_state_element_init = None

# store method of model element classes
MODEL_FILES = ['abstract_state_model', 'state_element_model']
old_abstract_state_model_init = None
old_state_element_model_init = None

# store method of extended ctrl class
CTRL_FILES = ['extended_controller']
old_extended_controller_init = None

# store method of gtkmvc element classes
GTKMVC_FILES = ['gtkmvc_view', 'gtkmvc_controller']
old_gtkmvc_view_init = gtkmvc.View.__init__
old_gtkmvc_model_mt_init = gtkmvc.ModelMT.__init__
old_gtkmvc_controller_init = gtkmvc.Controller.__init__

# store method of gaphas element classes
GAPHAS_FILES = ['gaphas_state_view', 'gaphas_extended_view', 'gaphas_port_view', 'gaphas_connection_view']
old_gaphas_port_view_init = None
old_gaphas_connection_view_init = None
old_gaphas_state_view_init = None
old_gaphas_extended_view_init = None

FILES = CORE_FILES + MODEL_FILES + CTRL_FILES + GAPHAS_FILES


def create_container_state(*args, **kargs):

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


def generate_sm_for_garbage_collector():
    ctr_state, dict_states = create_container_state()
    from rafcon.core.state_machine import StateMachine
    _ = StateMachine(ctr_state)


def get_log_elements(elements, with_prints=False, print_method=None):
    param_dict = get_param_dict()
    log_result_dict = {}
    for object_class, with_assert in elements:
        file_name = param_dict.get(object_class, None)[LOG_FILE_NAME_ID] if param_dict.get(object_class, None) else None
        if file_name is None:
            continue
        gen_file = os.path.join(RAFCON_TEMP_PATH_BASE, "{0}_{1}".format(file_name, GENERATION_LOG_FILE_APPENDIX))
        print gen_file
        with open(gen_file) as f:
            element_gen_file = f.readlines()
        element_gen_file = [line.replace('\n', '') for line in element_gen_file]

        # TODO find not deleted by checking gc for which are still existing if this is useful to know
        with_prints = False
        element_del_file = []

        # if element_name == 'state':
        for elem in element_gen_file:
            mem_id = elem.split(" ")

            if with_prints:
                _print_method = logger.error if with_assert else print_method
                if (mem_id[-2], mem_id[-1]) not in [(e.split(" ")[-2], e.split(" ")[-1]) for e in element_del_file]:
                    s = "{0} object still in memory: \n{1}\nDeleted are:\n{2}".format(file_name, elem,
                                                                                      '\n'.join(element_del_file))
                else:
                    s = None
                    # s = "{0} object destroyed: \n{1}\n".format(element_name, elem, ''.join(element_del_file))
                if s is not None:
                    if _print_method is not None:
                        _print_method(s)
                    else:
                        print s

        log_result_dict[file_name] = {'gen_file': element_gen_file, 'del_file': element_del_file}

    # print '\n'.join(["{0}: \n{1}\n{2}".format(element_name, log_files['gen_file'], log_files['del_file'])
    #                  for element_name, log_files in log_result_dict.iteritems()])

    return log_result_dict


def check_log_files(elements):
    files = []
    for element_name in elements:
        files.append(os.path.join(RAFCON_TEMP_PATH_BASE, "{0}_{1}".format(element_name, GENERATION_LOG_FILE_APPENDIX)))

    for file_path in files:
        print "check file: ", file_path
        if os.path.exists(file_path):
            print "exists before: ", file_path
        with open(file_path, 'a+'):
            pass
        if os.path.exists(file_path):
            print "exists after: ", file_path


def remove_log_files(elements):
    files = []
    for element_name in elements:
        files.append(os.path.join(RAFCON_TEMP_PATH_BASE, "{0}_{1}".format(element_name, GENERATION_LOG_FILE_APPENDIX)))
    print "REMOVE: \n{}".format('\n'.join([log_file_path for log_file_path in files]))
    for log_file_path in files:
        if os.path.exists(log_file_path):
            os.remove(log_file_path)


def create_models():
    import logging
    import rafcon.core.singleton
    from rafcon.core.state_machine import StateMachine
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.hierarchy_state import HierarchyState
    import rafcon.gui.singleton

    # global_gui_config.set_config_value('HISTORY_ENABLED', True)
    logger = log.get_logger(__name__)
    logger.setLevel(logging.VERBOSE)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)

    state1 = ExecutionState('State1', state_id='STATE1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2', state_id='STATE2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = HierarchyState(name='Nested', state_id='NESTED')
    state4.add_outcome('GoGo')
    state4.add_transition(state4.state_id, None, state4.state_id, 0)
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
    ctr_state.add_transition(ctr_state.state_id, None, state1.state_id, None)
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
    sm = StateMachine(ctr_state)
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id = sm.state_machine_id
    testing_utils.wait_for_gui()
    sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[sm.state_machine_id]
    # sm_m.history.fake = False
    # print "with_prints is: ", sm_m.history.with_prints
    # sm_m.history.with_prints = False
    return logger, sm_m, state_dict


def generate_graphs(target_object_s):
        try:
            import objgraph
        except ImportError:
            print "ImportError no generation of graph"
            return

        print "graph from object: ", target_object_s, id(target_object_s)

        if isinstance(target_object_s, list):
            target_object = target_object_s[0]
            folder_path = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, "..", "..",
                                       target_object.__class__.__name__)
            if os.path.exists(folder_path):
                shutil.rmtree(folder_path)
            for to in set(target_object_s):  # set used to additional avoid multiple identical graph generation
                generate_graphs(to)
        else:
            print "generate graph"
            target_object = target_object_s
            folder_path = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, "..", "..",
                                       target_object.__class__.__name__)
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)
            graph_file_name = os.path.join(folder_path, str(id(target_object)) + "_sample-graph.png")
            objgraph.show_backrefs(target_object,
                                   max_depth=7, extra_ignore=(), filter=None, too_many=10,
                                   highlight=None,
                                   extra_info=None, refcounts=True, shortnames=False,
                                   filename=graph_file_name)
            print "generate graph finished"


def check_existing_objects_of_kind(elements, print_method=None, ignored_objects=None, log_file=True,
                                   searched_type=None):
    # initial collect to avoid cross effects
    param_dict = get_param_dict()
    gc.collect()
    gc.collect()
    found_objects = []
    if ignored_objects is None:
        ignored_objects = []
    print_method = logger.warning if print_method is None else print_method
    if log_file:
        result_dict = get_log_elements(elements, with_prints=True, print_method=print_method)
    else:
        result_dict = {param_dict.get(object_class, None)[LOG_FILE_NAME_ID]
                       if param_dict.get(object_class, None) else str(object_class): {'gen_file': []}
                       for object_class, check_it in elements}
    for object_class, check_it in elements:
        name = param_dict.get(object_class, None)[LOG_FILE_NAME_ID] if param_dict.get(object_class, None) else None
        found_objects_of_kind = [o for o in gc.get_objects() if isinstance(o, object_class) and o not in ignored_objects]
        # # DEBUGGING work around used as exception -> stays in for possible reuse or will be cleaned up later ######
        # excepted_class_name = 'MainWindowController'
        # if any([o.__class__.__name__ == excepted_class_name for o in found_objects_of_kind]) and check_it:
        #     mw_ctrl_list = [o for o in found_objects_of_kind if o.__class__.__name__ == excepted_class_name]
        #     ignored_mw_ctrl_list = [o for o in ignored_objects if o.__class__.__name__ == excepted_class_name]
        #     found_objects_of_kind = [o for o in found_objects_of_kind if not o.__class__.__name__ == excepted_class_name]
        #     print "ignored main window controller ids", [id(o) for o in ignored_mw_ctrl_list]
        #     print "still existing main window controller ids", [id(o) for o in mw_ctrl_list]
        #     generate_graphs(mw_ctrl_list + ignored_objects)
        # # DEBUGGING work around end ###############################################################################
        found_objects += found_objects_of_kind

        if not len(found_objects_of_kind) == 0:
            collection_counts = [len(gc.get_referrers(o)) for o in found_objects_of_kind]
            class_types_found = set([o.__class__ for o in found_objects_of_kind])
            class_types_found = set(["{0}x {1}"
                                     "".format(len([o for o in found_objects_of_kind if isinstance(o, class_type)]),
                                               class_type.__name__)
                                     for class_type in class_types_found])
            print_method("of object of kind '{0}' have been generated {3} and there are {1} left over instances "
                         "with respective reference numbers of {2} and those types {4}"
                         "".format(object_class, len(found_objects_of_kind), collection_counts,
                                   len(result_dict[name]['gen_file']) if name else None, class_types_found))
        else:
            print "of object of kind '{0}' have been generated {2} and there are {1} left over instances " \
                  "".format(object_class, len(found_objects_of_kind),
                            len(result_dict[name]['gen_file']) if name else None)
        if check_it:
            assert len(found_objects_of_kind) == 0

    found_objects = list(set(found_objects))  # TODO why there are duplicated elements in

    # Use this commented lines to find you references of you classes which you wanna debug
    # BE AWARE every list generated in this script (above) is in the reference list, too!!!
    # -> target_object.g. target_objects and found_objects_of_kind
    if not searched_type:
        return found_objects

    target_objects = [o for o in found_objects if o.__class__.__name__ == searched_type]

    def get_classes_in_iter(it, name=True):
        if isinstance(it, dict):
            if name:
                return set(["{0}: {1}".format(key, element_in_iter.__class__.__name__)
                            for key, element_in_iter in it.iteritems()])
            else:
                return set(["{0}: {1}".format(key, element_in_iter.__class__)
                            for key, element_in_iter in it.iteritems()])
        else:
            if name:
                return set([element_in_iter.__class__.__name__ for element_in_iter in it])
            else:
                return set([element_in_iter.__class__ for element_in_iter in it])

    def print_referrer(referrer):
        if not hasattr(referrer, '__len__'):
            pprint(referrer)
        elif len(referrer) <= 3:
            pprint(["{1} with {0} elements: ".format(len(referrer), referrer.__class__.__name__),
                    referrer])
        else:
            pprint(["{1} with {0} elements of type: ".format(len(referrer), referrer.__class__.__name__),
                    get_classes_in_iter(referrer)])

    print "ignored_objects", ignored_objects
    print "found_objects", found_objects
    print "target_objects", target_objects

    if target_objects:
        generate_graphs(target_objects)
        return
        for index_target_object, target_object in enumerate(target_objects):
            print
            print
            print "# Referrers of #{0} {1}:".format(index_target_object + 1, searched_type), target_object

            # TODO the referrer prints should avoid to print the local variables list (by checking element keys)
            # TODO the referrer prints should avoid to print the list generated in this script (by inherit)
            target_object_referrers = gc.get_referrers(target_object)
            list_referrers = [referrer for referrer in target_object_referrers if hasattr(referrer, '__iter__')]

            print "## simple referrers", len(target_object_referrers)
            map(print_referrer, [referrer for referrer in target_object_referrers if referrer not in list_referrers])

            print "## list referrers"
            gc.collect()

            for referrer in list_referrers:
                print_referrer(referrer)
                second_instance_list_referrers = gc.get_referrers(referrer)
                print "### referrers of list referrer", len(second_instance_list_referrers)
                map(print_referrer, second_instance_list_referrers)
    # else:
    #     generate_graphs(found_objects)

    return found_objects


def run_model_construction():
    import rafcon.core.singleton
    import rafcon.gui.models.state

    logger, sm_m, state_dict = create_models()
    # root_state = sm_m.root_state

    c_state, state_dict = create_container_state()
    s_m = rafcon.gui.models.state.StateModel(c_state.states['STATE1'])
    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
    s_m.prepare_destruction()
    testing_utils.wait_for_gui()
    # s_m._reset_property_notification()

    # del logger
    # del sm_m
    # del state_dict
    return s_m


def run_simple_controller_construction():

    testing_utils.call_gui_callback(create_models)

    import rafcon.gui.singleton
    from gui.widget.test_states_editor import check_state_editor_models
    sm_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    testing_utils.call_gui_callback(check_state_editor_models, sm_m, sm_m.root_state)


def run_simple_modification_construction():

    testing_utils.call_gui_callback(create_models)

    import rafcon.gui.singleton
    from gui.widget.test_states_editor import check_state_editor_models
    sm_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    testing_utils.call_gui_callback(check_state_editor_models, sm_m, sm_m.root_state)
    import rafcon.gui.helpers.state
    list_exsisting_state_ids = sm_m.root_state.states.keys()

    print "%" * 50
    print "check before add_state"
    print "%" * 50

    testing_utils.call_gui_callback(rafcon.gui.helpers.state.add_state, sm_m.root_state,
                                    rafcon.gui.helpers.state.StateType.EXECUTION)
    print "%" * 50
    print "after first add"
    print "%" * 50
    testing_utils.call_gui_callback(rafcon.gui.helpers.state.add_state, sm_m.root_state,
                                    rafcon.gui.helpers.state.StateType.HIERARCHY)

    new_state_ids = [state_id for state_id, state_m in sm_m.root_state.states.iteritems()
                     if state_id not in list_exsisting_state_ids]
    for state_id in new_state_ids:
        testing_utils.call_gui_callback(sm_m.root_state.state.remove_state, state_id)
    print "%" * 50
    print "after deletes"
    print "%" * 50
    import time
    print "%" * 50
    print "do test menu bar"
    print "%" * 50
    import widget.test_menu_bar
    # TODO D-get this test also running with refresh
    widget.test_menu_bar.trigger_gui_signals(with_refresh=True)
    print "%" * 50
    print "do test complex actions, group & ungroup"
    print "%" * 50
    import test_complex_actions
    test_complex_actions.trigger_repetitive_group_ungroup()
    print "%" * 50
    print "do test ungroup"
    print "%" * 50
    import test_group_ungroup
    test_group_ungroup.trigger_ungroup_signals()
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)


def run_simple_execution_controller_construction():

    testing_utils.call_gui_callback(create_models)

    import rafcon.core.execution.execution_engine
    import rafcon.gui.singleton
    sm_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    testing_utils.call_gui_callback(rafcon.gui.singleton.state_machine_manager.__setattr__, "active_state_machine_id",
                                    sm_m.state_machine.state_machine_id)
    execution_engine = rafcon.gui.singleton.state_machine_execution_engine
    sm_execution_status = rafcon.core.execution.execution_engine.StateMachineExecutionStatus
    testing_utils.call_gui_callback(execution_engine.start)

    while execution_engine.status.execution_mode is not sm_execution_status.FINISHED:
        print "execution not finished yet: wait"
        time.sleep(0.01)
    testing_utils.call_gui_callback(rafcon.core.singleton.state_machine_manager.delete_all_state_machines)


def run_complex_controller_construction():

    testing_utils.call_gui_callback(create_models)

    # for a start load one of the type change tests to generate a lot of controllers which also close the GUI
    # from gui.widget.test_states_editor import create_models, MainWindowView, \
    #     MainWindowController, trigger_state_type_change_tests, gtk, threading
    from gui.widget.test_states_editor import trigger_state_type_change_tests
    trigger_state_type_change_tests(with_gui=True)


def patch_core_classes_with_log():

    import rafcon.core.states.state
    import rafcon.core.state_elements.state_element

    global old_state_init, old_state_element_init

    old_state_init = rafcon.core.states.state.State.__init__
    old_state_element_init = rafcon.core.state_elements.state_element.StateElement.__init__

    check_log_files(CORE_FILES)

    def state_init(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                   parent=None):
        self._name = name
        if state_id is None:
            self._state_id = rafcon.core.states.state.state_id_generator()
        else:
            self._state_id = state_id
        self.gen_time_stamp = int(round(time.time() * 1000))
        gen_file = os.path.join(RAFCON_TEMP_PATH_BASE, "{0}_{1}".format("state", GENERATION_LOG_FILE_APPENDIX))
        with open(gen_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(self, id(self), "state", self.gen_time_stamp))
        old_state_init(self, name, self._state_id, input_data_ports, output_data_ports, outcomes, parent)

    def state_element_init(self, parent=None):
        self.gen_time_stamp = int(round(time.time() * 1000))
        gen_file = os.path.join(RAFCON_TEMP_PATH_BASE, "{0}_{1}".format("state_element", GENERATION_LOG_FILE_APPENDIX))
        with open(gen_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(self, id(self), "state_element", self.gen_time_stamp))
        old_state_element_init(self, parent)

    rafcon.core.states.state.State.__init__ = state_init
    rafcon.core.state_elements.state_element.StateElement.__init__ = state_element_init


def un_patch_core_classes_from_log():
    import rafcon.core.states.state
    import rafcon.core.state_elements.state_element

    global old_state_init, old_state_element_init

    rafcon.core.states.state.State.__init__ = old_state_init
    rafcon.core.state_elements.state_element.StateElement.__init__ = old_state_element_init
    remove_log_files(CORE_FILES)


def patch_model_classes_with_log():

    import rafcon.gui.models.abstract_state
    import rafcon.gui.models.state_element

    global old_abstract_state_model_init, old_state_element_model_init

    old_abstract_state_model_init = rafcon.gui.models.abstract_state.AbstractStateModel.__init__
    old_state_element_model_init = rafcon.gui.models.state_element.StateElementModel.__init__

    check_log_files(MODEL_FILES)

    def abstract_state_model_init(self, state, parent=None, meta=None):
        self._state = None
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'abstract_state_model'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                      self.__kind, self.__gen_time_stamp))
        old_abstract_state_model_init(self, state, parent, meta)

    def state_element_model_init(self, parent, meta=None):
        self.parent = parent
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'state_element_model'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                  self.__kind, self.__gen_time_stamp))
        old_state_element_model_init(self, parent, meta)

    rafcon.gui.models.abstract_state.AbstractStateModel.__init__ = abstract_state_model_init
    rafcon.gui.models.state_element.StateElementModel.__init__ = state_element_model_init


def un_patch_model_classes_from_log():
    import rafcon.gui.models.abstract_state
    import rafcon.gui.models.state_element

    global old_abstract_state_model_init, old_state_element_model_init

    rafcon.gui.models.abstract_state.AbstractStateModel.__init__ = old_abstract_state_model_init
    rafcon.gui.models.state_element.StateElementModel.__init__ = old_state_element_model_init
    remove_log_files(MODEL_FILES)


def patch_ctrl_classes_with_log():

    # TODO maybe remove this again because the gtkmvc classes are covering this case
    import rafcon.gui.controllers.utils.extended_controller
    global old_extended_controller_init

    old_extended_controller_init = rafcon.gui.controllers.utils.extended_controller.ExtendedController.__init__

    check_log_files(CTRL_FILES)

    def extended_controller_init(self, model, view, spurious=None):
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'extended_controller'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                      self.__kind, self.__gen_time_stamp))
        if spurious is None:
            old_extended_controller_init(self, model, view)
        else:
            old_extended_controller_init(self, model, view, spurious)

    rafcon.gui.controllers.utils.extended_controller.ExtendedController.__init__ = extended_controller_init


def un_patch_ctrl_classes_from_log():
    import rafcon.gui.controllers.utils.extended_controller
    global old_extended_controller_init
    rafcon.gui.controllers.utils.extended_controller.ExtendedController.__init__ = old_extended_controller_init
    remove_log_files(CTRL_FILES)


def patch_gtkmvc_classes_with_log():

    global old_gtkmvc_view_init, old_gtkmvc_controller_init, old_gtkmvc_model_mt_init

    check_log_files(GTKMVC_FILES)

    def gtkmvc_view_init(self, glade=None, top=None, parent=None, builder=None):
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'gtkmvc_view'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                      self.__kind, self.__gen_time_stamp))
        old_gtkmvc_view_init(self, glade, top, parent, builder)

    def gtkmvc_controller_init(self, model, view, spurious=False, auto_adapt=False):
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'gtkmvc_controller'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                      self.__kind, self.__gen_time_stamp))
        old_gtkmvc_controller_init(self, model, view, spurious, auto_adapt)

    def gtkmvc_model_mt_init(self):
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'gtkmvc_model_mt'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                      self.__kind, self.__gen_time_stamp))
        old_gtkmvc_model_mt_init(self)

    gtkmvc.View.__init__ = gtkmvc_view_init
    # gtkmvc.ModelMT.__init__ = gtkmvc_model_mt_init
    gtkmvc.Controller.__init__ = gtkmvc_controller_init


def un_patch_gtkmvc_classes_from_log():
    global old_gtkmvc_view_init, old_gtkmvc_controller_init, old_gtkmvc_model_mt_init
    gtkmvc.View.__init__ = old_gtkmvc_view_init
    # gtkmvc.ModelMT.__init__ = old_gtkmvc_model_mt_init
    gtkmvc.Controller.__init__ = old_gtkmvc_controller_init
    remove_log_files(GTKMVC_FILES)


def patch_gaphas_classes_with_log():

    import rafcon.gui.mygaphas.view
    import rafcon.gui.mygaphas.items.ports
    from rafcon.gui.mygaphas.items.ports import SnappedSide
    import rafcon.gui.mygaphas.items.connection
    import rafcon.gui.mygaphas.items.state
    global old_gaphas_state_view_init, old_gaphas_extended_view_init, \
        old_gaphas_port_view_init, old_gaphas_connection_view_init

    old_gaphas_port_view_init = rafcon.gui.mygaphas.items.ports.PortView.__init__
    old_gaphas_connection_view_init = rafcon.gui.mygaphas.items.connection.ConnectionView.__init__
    old_gaphas_state_view_init = rafcon.gui.mygaphas.items.state.StateView.__init__
    old_gaphas_extended_view_init = rafcon.gui.mygaphas.view.ExtendedGtkView.__init__
    check_log_files(GAPHAS_FILES)

    def gaphas_extended_view_init(self, graphical_editor_v, state_machine_m, *args):
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'gaphas_extended_view'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                      self.__kind, self.__gen_time_stamp))
        old_gaphas_extended_view_init(self, graphical_editor_v, state_machine_m, *args)

    def gaphas_state_view_init(self, state_m, size, hierarchy_level):
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'gaphas_state_view'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                      self.__kind, self.__gen_time_stamp))
        old_gaphas_state_view_init(self, state_m, size, hierarchy_level)

    def gaphas_port_view_init(self, in_port, name=None, parent=None, side=SnappedSide.RIGHT):
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'gaphas_port_view'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                      self.__kind, self.__gen_time_stamp))
        old_gaphas_port_view_init(self, in_port, name, parent, side)

    def gaphas_connection_view_init(self, hierarchy_level):
        self.__gen_time_stamp = int(round(time.time() * 1000))
        self.__kind = 'gaphas_connection_view'
        self.__gen_log_file = os.path.join(RAFCON_TEMP_PATH_BASE, '{0}_{1}'.format(self.__kind,
                                                                                   GENERATION_LOG_FILE_APPENDIX))
        with open(self.__gen_log_file, 'a+') as f:
            f.write("RUN {2} of {0} {3} {1}\n".format(super(self.__class__, self).__str__(), id(self),
                                                      self.__kind, self.__gen_time_stamp))
        old_gaphas_connection_view_init(self, hierarchy_level)

    rafcon.gui.mygaphas.items.ports.PortView.__init__ = gaphas_port_view_init
    rafcon.gui.mygaphas.items.connection.ConnectionView.__init__ = gaphas_connection_view_init
    rafcon.gui.mygaphas.items.state.StateView.__init__ = gaphas_state_view_init
    rafcon.gui.mygaphas.view.ExtendedGtkView.__init__ = gaphas_extended_view_init


def un_patch_gaphas_classes_from_log():
    import rafcon.gui.mygaphas.view
    import rafcon.gui.mygaphas.items.ports
    import rafcon.gui.mygaphas.items.connection
    import rafcon.gui.mygaphas.items.state
    global old_gaphas_state_view_init, old_gaphas_extended_view_init, \
        old_gaphas_port_view_init, old_gaphas_connection_view_init
    rafcon.gui.mygaphas.items.ports.PortView.__init__ = old_gaphas_port_view_init
    rafcon.gui.mygaphas.items.connection.ConnectionView.__init__ = old_gaphas_connection_view_init
    rafcon.gui.mygaphas.items.state.StateView.__init__ = old_gaphas_state_view_init
    rafcon.gui.mygaphas.view.ExtendedGtkView.__init__ = old_gaphas_extended_view_init
    remove_log_files(GAPHAS_FILES)


def print_func(s):
    print s


LOG_FILE_NAME_ID = 0
PATCH_FUNCTION_ID = 1
UN_PATCH_FUNCTION_ID = 2


def get_param_dict():

    import rafcon.core
    import rafcon.gui.models.abstract_state
    import rafcon.gui.models.state_element
    import rafcon.gui.controllers.utils.extended_controller
    import rafcon.gui.mygaphas.items.ports
    import rafcon.gui.mygaphas.items.connection
    import rafcon.gui.mygaphas.items.state
    import rafcon.gui.mygaphas.view
    param_dict = {
                  rafcon.core.states.state.State:
                      ('state', patch_core_classes_with_log, un_patch_core_classes_from_log),
                  rafcon.core.state_elements.state_element.StateElement:
                      ('state_element', patch_core_classes_with_log, un_patch_core_classes_from_log),
                  rafcon.gui.models.abstract_state.AbstractStateModel:
                      ('abstract_state_model', patch_model_classes_with_log, un_patch_model_classes_from_log),
                  rafcon.gui.models.state_element.StateElementModel:
                      ('state_element_model', patch_model_classes_with_log, un_patch_model_classes_from_log),
                  rafcon.gui.controllers.utils.extended_controller.ExtendedController:
                      ('extended_controller', patch_ctrl_classes_with_log, un_patch_ctrl_classes_from_log),
                  gtkmvc.View:
                      ('gtkmvc_view', patch_gtkmvc_classes_with_log, un_patch_gtkmvc_classes_from_log),
                  # gtkmvc.ModelMT:
                  #     ('gtkmvc_model_mt', patch_gtkmvc_classes_with_log, un_patch_gtkmvc_classes_from_log),
                  gtkmvc.Controller:
                      ('gtkmvc_controller', patch_gtkmvc_classes_with_log, un_patch_gtkmvc_classes_from_log),
                  rafcon.gui.mygaphas.view.ExtendedGtkView:
                      ('gaphas_extended_view', patch_gaphas_classes_with_log, un_patch_gaphas_classes_from_log),
                  rafcon.gui.mygaphas.items.state.StateView:
                      ('gaphas_state_view', patch_gaphas_classes_with_log, un_patch_gaphas_classes_from_log),
                  rafcon.gui.mygaphas.items.ports.PortView:
                      ('gaphas_port_view', patch_gaphas_classes_with_log, un_patch_gaphas_classes_from_log),
                  rafcon.gui.mygaphas.items.connection.ConnectionView:
                      ('gaphas_connection_view', patch_gaphas_classes_with_log, un_patch_gaphas_classes_from_log),
                 }
    return param_dict


def run_patching(elements):
    param_dict = get_param_dict()
    f_set = set([param_dict[class_to_patch][PATCH_FUNCTION_ID] for class_to_patch, _ in elements
                 if class_to_patch in param_dict])
    for func in f_set:
        print "patch with: ", func
        func()


def run_un_patching(elements):
    param_dict = get_param_dict()
    f_set = set([param_dict[class_to_patch][UN_PATCH_FUNCTION_ID] for class_to_patch, check_it in elements
                 if class_to_patch in param_dict])
    for func in f_set:
        print "un-patch with: ", func
        func()


def test_core_destruct(caplog):
    testing_utils.dummy_gui(None)

    elements = [
                (rafcon.core.states.state.State, True),
                (rafcon.core.state_elements.state_element.StateElement, True),
                ]

    already_existing_objects = check_existing_objects_of_kind([(c, False) for c, check_it in elements],
                                                              print_func, log_file=False)

    testing_utils.initialize_environment_core()

    run_patching(elements)

    import core.test_states as basic_state_machines
    # basic_state_machines.test_create_state(caplog)
    # basic_state_machines.test_create_container_state(caplog)
    basic_state_machines.test_port_and_outcome_removal(caplog)
    # test
    generate_sm_for_garbage_collector()

    check_existing_objects_of_kind(elements, ignored_objects=already_existing_objects)

    run_un_patching(elements)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_model_and_core_destruct(caplog):

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False},
                                         gui_already_started=False)

    import rafcon.gui.models.abstract_state
    import rafcon.gui.models.state_element

    elements = [(rafcon.core.states.state.State, True),
                (rafcon.core.state_elements.state_element.StateElement, True),
                (rafcon.gui.models.abstract_state.AbstractStateModel, True),
                (rafcon.gui.models.state_element.StateElementModel, True),
                ]

    run_patching(elements)

    # if core test run before
    already_existing_objects = check_existing_objects_of_kind([(c, False) for c, check_it in elements],
                                                              print_func, log_file=False)

    run_model_construction()

    check_existing_objects_of_kind(elements, print_func, already_existing_objects)

    run_un_patching(elements)

    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_simple_model_and_core_destruct_with_gui(caplog):

    testing_utils.dummy_gui(None)

    import rafcon.gui.models.abstract_state
    import rafcon.gui.models.state_element
    import rafcon.gui.controllers.utils.extended_controller
    import rafcon.gui.mygaphas.view
    import rafcon.gui.mygaphas.items.state
    import rafcon.gui.mygaphas.items.connection
    import rafcon.gui.mygaphas.items.ports

    searched_class = rafcon.core.states.hierarchy_state.HierarchyState

    elements = [
                (rafcon.core.states.state.State, True),
                (rafcon.core.state_elements.state_element.StateElement, True),
                (rafcon.gui.models.abstract_state.AbstractStateModel, True),
                (rafcon.gui.models.state_element.StateElementModel, True),
                (rafcon.gui.controllers.utils.extended_controller.ExtendedController, True),
                (gtkmvc.View, True),
                (gtkmvc.Controller, True),
                (rafcon.gui.mygaphas.view.ExtendedGtkView, True),
                (rafcon.gui.mygaphas.items.connection.ConnectionView, True),
                (rafcon.gui.mygaphas.items.ports.PortView, True),
                (rafcon.gui.mygaphas.items.state.StateView, True),
                # (searched_class, False),
                ]
    run_setup_gui_destruct(caplog, elements, searched_class, run_simple_controller_construction,
                           # run_setup_gui_destruct(caplog, elements, searched_class, run_simple_modification_construction,
                           gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False})


def test_simple_execution_model_and_core_destruct_with_gui(caplog):

    testing_utils.dummy_gui(None)

    import rafcon.gui.models.abstract_state
    import rafcon.gui.models.state_element
    import rafcon.gui.controllers.utils.extended_controller

    searched_class = rafcon.core.states.execution_state.ExecutionState

    elements = [
                (rafcon.core.states.state.State, True),
                (rafcon.core.state_elements.state_element.StateElement, True),
                (rafcon.gui.models.abstract_state.AbstractStateModel, True),
                (rafcon.gui.models.state_element.StateElementModel, True),
                (rafcon.gui.controllers.utils.extended_controller.ExtendedController, True),
                (gtkmvc.View, True),
                # (gtkmvc.ModelMT, True),
                (gtkmvc.Controller, True),
                # (searched_class, False),
                ]
    run_setup_gui_destruct(caplog, elements, searched_class, run_simple_execution_controller_construction,
                           gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False})


def test_model_and_core_modification_history_destruct_with_gui(caplog):
    testing_utils.dummy_gui(None)

    import rafcon.gui.models.abstract_state
    import rafcon.gui.models.container_state
    import rafcon.gui.models.state_element
    import rafcon.gui.controllers.utils.extended_controller
    import rafcon.core.states.hierarchy_state
    import rafcon.core.states.execution_state
    import rafcon.gui.mygaphas.view
    import rafcon.gui.mygaphas.items.state
    import rafcon.gui.mygaphas.items.connection
    import rafcon.gui.mygaphas.items.ports

    searched_class = rafcon.core.states.hierarchy_state.HierarchyState
    # searched_class = rafcon.core.states.execution_state.ExecutionState
    searched_class = rafcon.gui.models.container_state.ContainerStateModel

    elements = [
                (rafcon.core.states.state.State, True),
                (rafcon.core.state_elements.state_element.StateElement, True),
                (rafcon.gui.models.abstract_state.AbstractStateModel, True),
                (rafcon.gui.models.state_element.StateElementModel, True),
                (rafcon.gui.controllers.utils.extended_controller.ExtendedController, True),
                (gtkmvc.View, True),
                (gtkmvc.Controller, True),
                (rafcon.gui.mygaphas.view.ExtendedGtkView, False),
                (rafcon.gui.mygaphas.items.connection.ConnectionView, False),
                (rafcon.gui.mygaphas.items.ports.PortView, False),
                (rafcon.gui.mygaphas.items.state.StateView, False),
                (searched_class, False),
                ]
    libraries = {"ros": os.path.join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                 "turtle_libraries": os.path.join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                 "generic": os.path.join(testing_utils.LIBRARY_SM_PATH, "generic")}
    run_setup_gui_destruct(caplog, elements, searched_class, run_simple_modification_construction,
                           gui_config={'AUTO_BACKUP_ENABLED': True, 'HISTORY_ENABLED': True}, libraries=libraries,
                           expected_warnings=0, expected_errors=1)
                           # gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False})


def run_copy_cut_and_paste():

    testing_utils.call_gui_callback(create_models)

    import rafcon.gui.helpers.state
    sm_m = rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model()
    from rafcon.gui.singleton import main_window_controller
    import rafcon.gui.singleton as gui_singletons
    from gui.widget.test_menu_bar import focus_graphical_editor_in_page
    menu_bar_controller = main_window_controller.get_controller("menu_bar_controller")
    state_machines_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')

    #########################################
    # copy tests
    #########################################

    # select state 1
    for sm_model in sm_m.root_state.states.values():
        if sm_model.state.name == "State1":
            selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
            testing_utils.call_gui_callback(selection.add, sm_model)
            print "select state: ", sm_model.state

    # focus correct page
    page_id = state_machines_ctrl.get_page_num(sm_m.state_machine.state_machine_id)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    focus_graphical_editor_in_page(page)

    # copy state 1
    testing_utils.call_gui_callback(menu_bar_controller.on_copy_selection_activate, None, None)
    print "copy state: ", sm_model.state

    # clear selection
    testing_utils.call_gui_callback(selection.clear)

    # select state 3
    for sm_model in sm_m.root_state.states.values():
        if sm_model.state.name == "State3":
            from rafcon.gui.models.container_state import ContainerStateModel
            assert isinstance(sm_model, ContainerStateModel)
            testing_utils.call_gui_callback(selection.add, sm_model)
            print "select state: ", sm_model.state
    # focus
    focus_graphical_editor_in_page(page)
    # paste state 1 into state 3
    testing_utils.call_gui_callback(menu_bar_controller.on_paste_clipboard_activate, None, None)
    print "pasted state into target state for the first time: ", sm_model.state

    # another time
    # select state 3
    for sm_model in sm_m.root_state.states.values():
        if sm_model.state.name == "State3":
            from rafcon.gui.models.container_state import ContainerStateModel
            assert isinstance(sm_model, ContainerStateModel)
            testing_utils.call_gui_callback(selection.add, sm_model)
            print "select state: ", sm_model.state
    # focus
    focus_graphical_editor_in_page(page)
    # paste state 1 into state 3
    testing_utils.call_gui_callback(menu_bar_controller.on_paste_clipboard_activate, None, None)
    print "pasted state into target state for the second time: ", sm_model.state

    #########################################
    # cut tests
    #########################################

    # select state 1
    for sm_model in sm_m.root_state.states.values():
        if sm_model.state.name == "State1":
            state1 = sm_model.state
            selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
            testing_utils.call_gui_callback(selection.add, sm_model)
            print "select state: ", sm_model.state

    # focus correct page
    page_id = state_machines_ctrl.get_page_num(sm_m.state_machine.state_machine_id)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    focus_graphical_editor_in_page(page)

    # cut state 1
    testing_utils.call_gui_callback(menu_bar_controller.on_cut_selection_activate, None, None)
    print "cut state: ", sm_model.state

    # destroy state test
    # print "%" * 20, "before  ", "%" * 20
    # testing_utils.call_gui_callback(sm_m.root_state.state.remove_state, state1.state_id)
    # print "%" * 20, "after  ", "%" * 20

    # clear selection
    testing_utils.call_gui_callback(selection.clear)

    # select state 3
    for sm_model in sm_m.root_state.states.values():
        if sm_model.state.name == "State3":
            from rafcon.gui.models.container_state import ContainerStateModel
            assert isinstance(sm_model, ContainerStateModel)
            testing_utils.call_gui_callback(selection.add, sm_model)
            print "select state: ", sm_model.state

    # focus
    focus_graphical_editor_in_page(page)

    # paste state 1 into state 3
    testing_utils.call_gui_callback(menu_bar_controller.on_paste_clipboard_activate, None, None)
    print "paste state into target state: ", sm_model.state

    # import time
    # time.sleep(10.0)
    testing_utils.call_gui_callback(rafcon.core.singleton.state_machine_manager.delete_all_state_machines)


def test_copy_paste_with_modification_history_destruct_with_gui(caplog):

    testing_utils.dummy_gui(None)

    import rafcon.gui.models.abstract_state
    import rafcon.gui.models.state_element
    import rafcon.gui.controllers.utils.extended_controller
    import rafcon.core.states.hierarchy_state

    # searched_class = rafcon.core.states.execution_state.ExecutionState
    # searched_class = rafcon.gui.models.container_state.ContainerStateModel
    searched_class = rafcon.gui.models.state.AbstractStateModel
    # searched_class = rafcon.core.state_elements.data_flow.DataFlow
    # searched_class = rafcon.core.state_elements.transition.Transition
    # searched_class = rafcon.gui.models.transition.TransitionModel

    elements = [
                (rafcon.core.states.state.State, True),
                (rafcon.core.state_elements.state_element.StateElement, True),
                (rafcon.gui.models.abstract_state.AbstractStateModel, True),
                (rafcon.gui.models.state_element.StateElementModel, True),
                (rafcon.gui.controllers.utils.extended_controller.ExtendedController, True),
                (gtkmvc.View, True),
                (gtkmvc.Controller, True),
                (searched_class, False),
                ]
    run_setup_gui_destruct(caplog, elements, searched_class, run_copy_cut_and_paste,
                           gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False}, expected_warnings=0)


def test_complex_model_and_core_destruct_with_gui(caplog):

    testing_utils.dummy_gui(None)

    import rafcon.gui.models.abstract_state
    import rafcon.gui.models.state_element
    import rafcon.gui.controllers.utils.extended_controller
    import rafcon.core.states.hierarchy_state

    import rafcon.gui.models.container_state
    # searched_class = rafcon.core.states.hierarchy_state.HierarchyState
    searched_class = rafcon.gui.models.container_state.ContainerStateModel

    elements = [
                (rafcon.core.states.state.State, True),
                (rafcon.core.state_elements.state_element.StateElement, True),
                (rafcon.gui.models.abstract_state.AbstractStateModel, True),
                (rafcon.gui.models.state_element.StateElementModel, True),
                (rafcon.gui.controllers.utils.extended_controller.ExtendedController, True),
                (gtkmvc.View, True),
                (gtkmvc.Controller, True),
                (searched_class, False),
                ]
    run_setup_gui_destruct(caplog, elements, searched_class, run_complex_controller_construction,
                           gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False})


def run_setup_gui_destruct(caplog, elements, searched_class, func, gui_config, libraries=None, expected_warnings=0, expected_errors=0):
    # if core test run before
    import rafcon.gui.singleton
    rafcon.gui.singleton.main_window_controller = None
    already_existing_objects = check_existing_objects_of_kind([(c, False) for c, check_it in elements],
                                                              print_func, log_file=False)

    # TODO make it fully working and later activate modification history and auto backup
    testing_utils.run_gui(gui_config=gui_config, libraries=libraries)

    run_patching(elements)

    exception_during_test_method = None

    try:
        print "%" * 50
        print "before test function print"
        print "%" * 50
        func()
    except Exception as e:
        exception_during_test_method = e
        raise
    # finally:
    testing_utils.close_gui()
    rafcon.gui.singleton.main_window_controller = None  # could be moved to the testing_utils but is not needed
    if exception_during_test_method:
        raise exception_during_test_method
    print "%" * 50
    print "check for existing objects print"
    print "%" * 50
    testing_utils.shutdown_environment(caplog=caplog, expected_warnings=expected_warnings, expected_errors=expected_errors)
    check_existing_objects_of_kind(elements, print_func, ignored_objects=already_existing_objects,
                                   searched_type=searched_class.__name__)
    run_un_patching(elements)


if __name__ == '__main__':
    # testing_utils.dummy_gui(None)
    # test_core_destruct(None)
    # test_model_and_core_destruct(None)
    # test_simple_model_and_core_destruct_with_gui(None)
    # test_simple_execution_model_and_core_destruct_with_gui(None)
    # test_model_and_core_modification_history_destruct_with_gui(None)
    test_copy_paste_with_modification_history_destruct_with_gui(None)
    # test_complex_model_and_core_destruct_with_gui(None)
    # import pytest
    # pytest.main(['-s', __file__])

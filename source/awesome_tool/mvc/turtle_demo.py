import logging
import sys
import os

import gtk

from utils import log
from mvc.controllers import MainWindowController
from mvc.views import LoggingView, MainWindowView
from mvc.models import ContainerStateModel, GlobalVariableManagerModel, ExternalModuleManagerModel
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.execution_state import ExecutionState
from statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from statemachine.external_modules.external_module import ExternalModule
import statemachine.singleton
from mvc.models.state_machine_manager import StateMachineManagerModel
from statemachine.state_machine import StateMachine


import gobject
gobject.threads_init()

def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)


def create_models():
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    #logging.getLogger('gtkmvc').setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)
    logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)

    external_module_manager_model = ExternalModuleManagerModel()

    global_var_manager_model = GlobalVariableManagerModel()
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return logger, global_var_manager_model, external_module_manager_model

def create_turtle_statemachine():
     ########################################################
    # state machine creation start
    ########################################################

    #statemachine.singleton.state_machine_execution_engine.start()

    turtle_demo_state = BarrierConcurrencyState("TurtleDemo", path="../../test_scripts/turtle_demo",
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

    ########################################################
    # follower turtle bot
    ########################################################

    # the food turtles are created in the entry script

    follower_turtle_bot_hierarchy_state = HierarchyState("Follower Turtle Bot", path="../../test_scripts/turtle_demo",
                                                         filename="follower_turtle_bot_hierarchy_state.py")
    follower_turtle_bot_hierarchy_state.add_outcome("Success", 0)
    turtle_demo_state.add_state(follower_turtle_bot_hierarchy_state)
    follower_turtle_bot_hierarchy_state.add_scoped_variable("demo_scoped_variable", "str", "demo_default_value")

    # create bot turtle

    create_bot_turtle_state = ExecutionState("create bot turtle", path="../../test_scripts/turtle_demo",
                                         filename="create_bot_turtle.py")
    create_bot_turtle_state.add_outcome("Success", 0)
    follower_turtle_bot_hierarchy_state.add_state(create_bot_turtle_state)
    follower_turtle_bot_hierarchy_state.set_start_state(create_bot_turtle_state.state_id)

    # check for food and follow

    check_food_and_follow_state = PreemptiveConcurrencyState("Check Food and Follow",
                                                             path="../../test_scripts/turtle_demo",
                                                             filename="check_food_and_follow.py")
    follower_turtle_bot_hierarchy_state.add_state(check_food_and_follow_state)
    follower_turtle_bot_hierarchy_state.add_transition(create_bot_turtle_state.state_id, 0,
                                                       check_food_and_follow_state.state_id, None)
    check_food_and_follow_state.add_outcome("Eat", 0)

    #check for food

    check_for_food_hierarchy = HierarchyState("Check for food hierarchy", path="../../test_scripts/turtle_demo",
                                              filename="check_for_food_hierarchy.py")
    check_for_food_hierarchy_output = \
        check_for_food_hierarchy.add_output_data_port("target turtle", "str", "default_turtle_value")
    check_for_food_hierarchy.add_outcome("Success", 0)
    check_for_food = ExecutionState("Check for food", path="../../test_scripts/turtle_demo",
                                    filename="check_for_food.py")
    check_for_food_hierarchy.add_state(check_for_food)
    check_for_food_hierarchy.set_start_state(check_for_food.state_id)
    check_for_food.add_outcome("Success", 0)
    check_for_food.add_outcome("Nothing found", 1)
    check_for_food_output = check_for_food.add_output_data_port("target turtle", "str", "default_turtle_value")

    check_for_food_hierarchy.add_transition(check_for_food.state_id, 0, None, 0)
    check_for_food_hierarchy.add_transition(check_for_food.state_id, 1, check_for_food.state_id, None)
    # pass turtle to the outer hierarchy state
    check_for_food_hierarchy.add_data_flow(check_for_food.state_id, check_for_food_output,
                                              check_for_food_hierarchy.state_id, check_for_food_hierarchy_output)

    check_food_and_follow_state.add_state(check_for_food_hierarchy)
    # pass turtle to the
    check_for_food_and_follow_output = check_food_and_follow_state.add_output_data_port("target turtle", "str")
    check_food_and_follow_state.add_transition(check_for_food_hierarchy.state_id, 0, None, 0)
    check_food_and_follow_state.add_data_flow(check_for_food_hierarchy.state_id, check_for_food_hierarchy_output,
                                              check_food_and_follow_state.state_id, check_for_food_and_follow_output)

    #follow user turtle

    follow_user_turtle_hierarchy = HierarchyState("Follow User Turtle Hierarchy",
                                                  path="../../test_scripts/turtle_demo",
                                                  filename="follow_user_turtle_hierarchy.py")
    check_food_and_follow_state.add_state(follow_user_turtle_hierarchy)

    check_user_turtle_position = ExecutionState("Check user turtle position",
                                                path="../../test_scripts/turtle_demo",
                                                filename="check_user_turtle_position.py")
    check_user_turtle_position.add_outcome("Success", 0)
    check_position_output = check_user_turtle_position.add_output_data_port("user turtle position", "dict")
    calculate_position_difference = ExecutionState("Calculate position difference",
                                                   path="../../test_scripts/turtle_demo",
                                                   filename="calculate_position_difference.py")
    calculate_position_difference.add_outcome("Success", 0)
    calculate_difference_input = calculate_position_difference.add_input_data_port("user turtle position", "dict")
    calculate_difference_output = calculate_position_difference.add_output_data_port("distance to user turtle", "dict")
    move_difference = ExecutionState("Move difference", path="../../test_scripts/turtle_demo",
                                     filename="move_difference.py")
    move_difference.add_outcome("Success", 0)
    move_difference_input = move_difference.add_input_data_port("distance to user turtle", "dict")

    follow_user_turtle_hierarchy.add_state(check_user_turtle_position)
    follow_user_turtle_hierarchy.set_start_state(check_user_turtle_position.state_id)
    follow_user_turtle_hierarchy.add_state(calculate_position_difference)
    follow_user_turtle_hierarchy.add_state(move_difference)


    follow_user_turtle_hierarchy.add_transition(check_user_turtle_position.state_id, 0,
                                                calculate_position_difference.state_id, None)
    follow_user_turtle_hierarchy.add_transition(calculate_position_difference.state_id, 0,
                                                move_difference.state_id, None)
    follow_user_turtle_hierarchy.add_transition(move_difference.state_id, 0,
                                                check_user_turtle_position.state_id, None)

    follow_user_turtle_hierarchy.add_data_flow(check_user_turtle_position.state_id, check_position_output,
                                               calculate_position_difference.state_id, calculate_difference_input)
    follow_user_turtle_hierarchy.add_data_flow(calculate_position_difference.state_id, calculate_difference_output,
                                               move_difference.state_id, move_difference_input)

    # eat
    eat = ExecutionState("Eat", path="../../test_scripts/turtle_demo", filename="eat.py")
    follower_turtle_bot_hierarchy_state.add_state(eat)
    eat_input = eat.add_input_data_port("turtle to eat", "str")
    follower_turtle_bot_hierarchy_state.add_data_flow(check_food_and_follow_state.state_id,
                                                      check_for_food_and_follow_output,
                                                      eat.state_id, eat_input)
    eat.add_outcome("Finished Eating", 0)

    follower_turtle_bot_hierarchy_state.add_transition(eat.state_id, 0, check_food_and_follow_state.state_id, 0)
    follower_turtle_bot_hierarchy_state.add_transition(check_food_and_follow_state.state_id, 0, eat.state_id, 0)
    return turtle_demo_state


def run_turtle_demo():

    turtle_demo_state = create_turtle_statemachine()

    ####################################################################
    # comment these lines out to save the modified statemachine
    #####################################################################
    # statemachine.singleton.global_storage.base_path = "../../test_scripts/turtle_demo_state_machine"
    # statemachine.singleton.global_storage.save_statemachine_as_yaml(turtle_demo_state)
    # print "#################################################################"
    # print "load turtle state machine from disk"
    # print "#################################################################"
    [turtle_demo_state, version, creation_time] = \
        statemachine.singleton.global_storage.load_statemachine_from_yaml("../../test_scripts/turtle_demo_state_machine")

    statemachine.singleton.library_manager.initialize()
    logging_view = LoggingView()
    setup_logger(logging_view)
    [logger, gvm_model, emm_model] = create_models()
    main_window_view = MainWindowView(logging_view)
    state_machine = StateMachine(turtle_demo_state)
    statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = StateMachineManagerModel(statemachine.singleton.state_machine_manager)

    # load the meta data for the state machine
    sm_manager_model.get_active_state_machine_model().root_state.load_meta_data_for_state()

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, emm_model, gvm_model, editor_type="ld")
    #main_window_controller = MainWindowController(sm_manager_model, main_window_view, emm_model, gvm_model)

    #graphical_editor_view = SingleWidgetWindowView(GraphicalEditorView, title="Graphical Editor", pos=1)
    #graphical_editor_ctrl = SingleWidgetWindowController(ctr_model, graphical_editor_view, GraphicalEditorController)

    # external modules
    ros_module = ExternalModule(name="ros", module_name="ros_external_module", class_name="RosModule")
    statemachine.singleton.external_module_manager.add_external_module(ros_module)
    statemachine.singleton.external_module_manager.external_modules["ros"].connect([])
    statemachine.singleton.external_module_manager.external_modules["ros"].start()

    user_input_module = ExternalModule(name="user_input", module_name="user_input_external_module", class_name="UserInput")
    statemachine.singleton.external_module_manager.add_external_module(user_input_module)
    statemachine.singleton.external_module_manager.external_modules["user_input"].connect([])
    #statemachine.singleton.external_module_manager.external_modules["user_input"].start()

    # dual_gtk_window = DualGTKWindow()

    print "before main loop"

    gtk.main()
    logger.debug("Gtk main loop exited!")
    # save the meta data for the state machine
    sm_manager_model.get_active_state_machine_model().root_state.store_meta_data_for_state()
    statemachine.singleton.state_machine_manager.get_active_state_machine().root_state.join()


class DualGTKWindow:

    def __init__(self):
        self.external_module_stopped = False
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.set_title("DualGTKWindoTest!")
        self.window.connect("delete_event", self.delete_event)
        self.window.set_border_width(10)
        self.box1 = gtk.HBox(False, 0)
        self.window.add(self.box1)
        self.button1 = gtk.Button("Button 1")
        self.button1.connect("clicked", self.callback, "button 1")
        self.box1.pack_start(self.button1, True, True, 0)
        self.button1.show()
        self.button2 = gtk.Button("Button 2")
        self.button2.connect("clicked", self.callback, "button 2")
        self.box1.pack_start(self.button2, True, True, 0)
        self.button2.show()
        self.box1.show()
        self.window.show()

    def callback(self, widget, data):
        print "Hello again - %s was pressed" % data
        if not self.external_module_stopped:
            statemachine.singleton.external_module_manager.external_modules["user_input"].stop()
            self.external_module_stopped = True

    def delete_event(self, widget, event, data=None):
        gtk.main_quit()
        return False


if __name__ == '__main__':
    cur_path = os.path.abspath(os.path.dirname(__file__))
    test_script_path = os.path.join(cur_path, os.pardir, os.pardir, 'test_scripts')
    sys.path.insert(1, test_script_path)
    #print sys.path
    run_turtle_demo()

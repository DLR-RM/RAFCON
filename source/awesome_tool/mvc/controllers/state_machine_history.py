import gtk
import gobject
import yaml

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.models import ContainerStateModel
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.mvc.models.state_machine import StateMachineModel
from awesome_tool.utils import log

from awesome_tool.statemachine.states.state import State, DataPort
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.states.library_state import LibraryState
from awesome_tool.statemachine.data_flow import DataFlow
from awesome_tool.statemachine.transition import Transition

from awesome_tool.mvc.models.state import StateModel
from awesome_tool.mvc.models.container_state import ContainerState
from awesome_tool.mvc.models.container_state import ContainerState

logger = log.get_logger(__name__)

# TODO Comment


class StateMachineHistoryController(ExtendedController):

    def __init__(self, model, view):
        """Constructor
        :param model StateMachineModel should be exchangeable
        """
        assert isinstance(model, StateMachineManagerModel)

        ExtendedController.__init__(self, model, view)
        self.view_is_registered = False

        # Nr, version_id, Method, Instance, Details, model
        self.list_store = gtk.ListStore(str, str, str, str, str, str, gobject.TYPE_PYOBJECT)
        if view is not None:
            view['state_machine_history_tree'].set_model(self.list_store)
        #view.set_hover_expand(True)

        self.__my_selected_sm_id = None
        self._selected_sm_model = None

        self.count = 0
        self.doing_update = False
        self.no_cursor_observation = False

        self.register()

    @ExtendedController.observe("selected_state_machine_id", assign=True)
    def state_machine_manager_notification(self, model, property, info):
        self.register()

    def register(self):
        """
        Change the state machine that is observed for new selected states to the selected state machine.
        :return:
        """
        # logger.debug("StateMachineEditionChangeHistory register state_machine old/new sm_id %s/%s" %
        #              (self.__my_selected_sm_id, self.model.selected_state_machine_id))

        # relieve old models
        if self.__my_selected_sm_id is not None:  # no old models available
            self.relieve_model(self._selected_sm_model.history)

        if self.model.selected_state_machine_id is not None:

            # set own selected state machine id
            self.__my_selected_sm_id = self.model.selected_state_machine_id

            # observe new models
            self._selected_sm_model = self.model.state_machines[self.__my_selected_sm_id]
            self.observe_model(self._selected_sm_model.history)
            self.update(None, None, None)
        else:
            self.__my_selected_sm_id = None
            self._selected_sm_model = None

    def register_view(self, view):
        self.view['state_machine_history_tree'].connect('cursor-changed', self.on_cursor_changed)
        self.view_is_registered = True

    def register_adapters(self):
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param awesome_tool.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("undo", self.undo)
        shortcut_manager.add_callback_for_action("redo", self.redo)

    def on_cursor_changed(self, widget):
        if self.no_cursor_observation:
            return
        (model, row) = self.view['state_machine_history_tree'].get_selection().get_selected()
        self.no_cursor_observation = True
        if not self.doing_update:
            logger.debug("The view jumps to the selected history element that would be situated on a right click menu in future")
            # get selected element
            logger.debug("%s, %s" % (model, row))
            for elem in model[row]:
                logger.debug(str(elem))

            # take version_id
            version_id = model[row][1]
            # do recovery
            self.doing_update = True
            self._selected_sm_model.history.recover_specific_version(version_id)
            self.doing_update = False
            self.update(None, None, None)
        self.no_cursor_observation = False


    def undo(self, key_value, modifier_mask):
        logger.debug("Run history UNDO")
        self._selected_sm_model.history.undo()
        # print len(self._selected_sm_model.history.changes.single_trail_history()), \
        #     self._selected_sm_model.history.changes.single_trail_history()
        self.update(None, None, None)

    def redo(self, key_value, modifier_mask):
        logger.debug("Run history REDO")
        self._selected_sm_model.history.redo()
        # print len(self._selected_sm_model.history.changes.single_trail_history()), \
        #     self._selected_sm_model.history.changes.single_trail_history()
        self.update(None, None, None)

    @ExtendedController.observe("changes", after=True)
    def update(self, model, prop_name, info):
        # print "History changed %s\n%s\n%s" % (model, prop_name, info)
        if self._selected_sm_model.history.fake or info is not None and not info.method_name == "insert_action":
            return
        self.doing_update = True
        self.list_store.clear()
        self.count = 0
        for action in self._selected_sm_model.history.changes.single_trail_history():
            # if action.before_info['kwargs']:
            #     self.new_change(action.before_model, action.before_prop_name, action.before_info)
            # else:
            # self.new_change(action.before_model, action.before_prop_name, action.before_info)
            if not 'method_name' in action.before_info:
                logger.warning("Found no method_name in before_info")
                method_name = None
            else:
                method_name = action.before_info['method_name']

            if not 'instance' in action.before_info:
                logger.warning("Found no instance in before_info")
                inst = None
            else:
                inst = action.before_info['instance']
            self.new_change(action.before_model, action.before_prop_name,
                            method_name, inst, info, action.version_id)

            # self.new_change(action.before_model, action.before_prop_name,
            #                 action.before_info['method_name'], action.before_info['instance'], info)

        # set selection of Tree
        row_number = self._selected_sm_model.history.changes.trail_pointer
        if len(self.list_store) > 0:
            self.view['state_machine_history_tree'].set_cursor(len(self.list_store) - 1 - row_number)

        # set colors of Tree
        # - is state full and all element which are open to be re-done gray
        self.doing_update = False

    def new_change(self, model, prop_name, method_name, instance, info, version_id):
        # Nr, Instance, Method, Details, model
        if not self._selected_sm_model.history.locked:
            row_number = self._selected_sm_model.history.changes.trail_pointer
            if len(self.list_store) <= row_number:
                foreground = "white"
                # foreground = "green"
            else:
                # foreground = "gray"
                foreground = "#707070"
            self.list_store.prepend((self.count,
                                     version_id,  # '',  # version
                                     method_name,
                                     instance,
                                     info,
                                     foreground,
                                     model))
            self.count += 1

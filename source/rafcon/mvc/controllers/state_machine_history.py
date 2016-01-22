import gobject

from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.models import ContainerStateModel
from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.mvc.models.state_machine import StateMachineModel
from rafcon.utils import log

from rafcon.statemachine.states.state import State, DataPort
from rafcon.statemachine.outcome import Outcome
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.data_flow import DataFlow
from rafcon.statemachine.transition import Transition

from rafcon.mvc.models.state import StateModel
from rafcon.mvc.models.container_state import ContainerState
from rafcon.mvc.models.container_state import ContainerState

from rafcon.mvc import singleton as mvc_singleton

logger = log.get_logger(__name__)

# TODO Comment
import gtk


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
            view['history_tree'].set_model(self.list_store)
        # view.set_hover_expand(True)

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
            if self.__my_selected_sm_id is not None:
                self.list_store.clear()
            self.__my_selected_sm_id = None
            self._selected_sm_model = None

    def register_view(self, view):
        view['history_tree'].connect('cursor-changed', self.on_cursor_changed)
        view['undo_button'].connect('clicked', self.on_undo_button_clicked)
        view['redo_button'].connect('clicked', self.on_redo_button_clicked)
        view['reset_button'].connect('clicked', self.on_reset_button_clicked)
        self.view_is_registered = True

    def register_adapters(self):
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("undo", self.undo)
        shortcut_manager.add_callback_for_action("redo", self.redo)

    def on_undo_button_clicked(self, widget, event=None):
        self.undo(None, None)

    def on_redo_button_clicked(self, widget, event=None):
        self.redo(None, None)

    def on_reset_button_clicked(self, widget, event=None):
        # logger.debug("do reset")
        self._selected_sm_model.history.changes.reset()

    def on_cursor_changed(self, widget):
        if self.no_cursor_observation:
            return
        (model, row) = self.view['history_tree'].get_selection().get_selected()
        self.no_cursor_observation = True
        if not self.doing_update:
            logger.debug(
                "The view jumps to the selected history element that would be situated on a right click menu in future")

            version_id = model[row][1]

            # do recovery
            self.doing_update = True
            self._selected_sm_model.history.recover_specific_version(version_id)
            self.doing_update = False
            self.update(None, None, None)
        self.no_cursor_observation = False

    def undo(self, key_value, modifier_mask):
        for key, tab in mvc_singleton.main_window_controller.get_controller('states_editor_ctrl').tabs.iteritems():
            if tab['controller'].get_controller('source_ctrl') is not None and \
                    tab['controller'].get_controller('source_ctrl').view.textview.has_focus():
                # print tab['controller'].get_controller('source_ctrl').view.textview.get_buffer().can_undo(), key
                if tab['controller'].get_controller('source_ctrl').view.textview.get_buffer().can_undo():
                    return False
        self._selected_sm_model.history.undo()
        return True

    def redo(self, key_value, modifier_mask):
        for key, tab in mvc_singleton.main_window_controller.get_controller('states_editor_ctrl').tabs.iteritems():
            if tab['controller'].get_controller('source_ctrl') is not None and \
                    tab['controller'].get_controller('source_ctrl').view.textview.has_focus():
                # print tab['controller'].get_controller('source_ctrl').view.textview.get_buffer().can_redo(), key
                if tab['controller'].get_controller('source_ctrl').view.textview.get_buffer().can_redo():
                    return False
        self._selected_sm_model.history.redo()
        return True

    @ExtendedController.observe("changes", after=True)
    def update(self, model, prop_name, info):
        # logger.debug("History changed %s\n%s\n%s" % (model, prop_name, info))
        if self._selected_sm_model.history.fake or \
                info is not None and info.method_name not in ["insert_action", "undo", "redo", "reset"]:
            return
        self.doing_update = True
        self.list_store.clear()
        self.count = 0
        for action in self._selected_sm_model.history.changes.single_trail_history():
            # if action.before_info['kwargs']:
            #     self.new_change(action.before_model, action.before_prop_name, action.before_info)
            # else:
            # self.new_change(action.before_model, action.before_prop_name, action.before_info)
            if 'method_name' not in action.before_overview['info'][-1]:
                logger.warning("Found no method_name in before_info %s" % action.before_overview['info'][-1])
                method_name = None
            else:
                method_name = action.before_overview['method_name'][-1]

            if 'instance' not in action.before_overview['info'][-1]:
                logger.warning("Found no instance in before_info %s" % action.before_overview['info'][-1])
                inst = None
            else:
                inst = action.before_overview['instance'][-1]
            self.new_change(action.before_overview['model'][-1], action.before_overview['prop_name'][-1],
                            method_name, inst, info, action.version_id)

            # self.new_change(action.before_model, action.before_prop_name,
            #                 action.before_info['method_name'], action.before_info['instance'], info)

        # set selection of Tree
        row_number = self._selected_sm_model.history.changes.trail_pointer
        if len(self.list_store) > 0:
            self.view['history_tree'].set_cursor(len(self.list_store) - 1 - row_number)

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

import gtk
from gtk.gdk import keyval_name
from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from gtkmvc import Model

from awesome_tool.statemachine.enums import StateType

from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from awesome_tool.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState, DeciderState
from awesome_tool.statemachine.states.library_state import LibraryState


from awesome_tool.utils import log
logger = log.get_logger(__name__)


class StateOverviewController(ExtendedController, Model):
    """Controller handling the view of properties/attributes of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.source_editor.SourceEditorView` and the properties of the
    :class:`mvc.models.state.StateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param awesome_tool.mvc.models.StateModel model: The state model containing the data
    :param awesome_tool.mvc.views.SourceEditorView view: The GTK view showing the data as a table
    """

    # TODO Missing functions

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)

        self.state_types_dict = {}

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        # prepare State Type Change ComboBox
        if isinstance(self.model.state, DeciderState):
            # logger.info(str(StateType))
            self.state_types_dict[str(StateType.DECIDER_STATE).split('.')[1]] = {
                'Enum': StateType.DECIDER_STATE, 'class': DeciderState}
        else:
            self.state_types_dict[str(StateType.EXECUTION).split('.')[1]] = {
                'Enum': StateType.EXECUTION, 'class': ExecutionState}
            self.state_types_dict[str(StateType.HIERARCHY).split('.')[1]] = {
                'Enum': StateType.HIERARCHY, 'class': HierarchyState}
            self.state_types_dict[str(StateType.BARRIER_CONCURRENCY).split('.')[1]] = {
                'Enum': StateType.BARRIER_CONCURRENCY, 'class': BarrierConcurrencyState}
            self.state_types_dict[str(StateType.PREEMPTION_CONCURRENCY).split('.')[1]] = {
                'Enum': StateType.PREEMPTION_CONCURRENCY, 'class': PreemptiveConcurrencyState}

        view['entry_name'].connect('focus-out-event', self.change_name)
        view['entry_name'].connect('key-press-event', self.check_for_enter)
        if self.model.state.name:
            view['entry_name'].set_text(self.model.state.name)
        view['label_id_value'].set_text(self.model.state.state_id)

        if self.model.parent is None:
            self.view['is_start_state_checkbutton'].hide()

        l_store = gtk.ListStore(str)
        combo = gtk.ComboBox()
        combo.set_focus_on_click(False)
        combo.set_model(l_store)
        cell = gtk.CellRendererText()
        combo.pack_start(cell, True)
        combo.add_attribute(cell, 'text', 0)
        combo.show_all()
        view['type_viewport'].add(combo)
        view['type_viewport'].show()

        # Prepare LAbel for state_name -> Library states cannot be changed
        if isinstance(self.model.state, LibraryState):
            l_store.prepend(['LIBRARY'])
            self.view['library_name'].set_text(self.model.state.library_name)
            combo.set_sensitive(False)
        else:
            self.view['label_library_name'].hide()
            self.view['library_name'].hide()
            for key, value in self.state_types_dict.iteritems():
                if value['class'] == type(self.model.state):
                    l_store.prepend([key])
                else:
                    l_store.append([key])

        combo.set_active(0)
        view['type_combobox'] = combo
        view['type_combobox'].connect('changed', self.change_type)

        # Prepare "is start state check button"
        # has_no_start_state_state_types = [BarrierConcurrencyState, PreemptiveConcurrencyState]
        if isinstance(self.model.state, DeciderState):  # \
            # has to re-initiated if parent becomes HierarchchyState
            #     or self.model.parent is not None and type(self.model.parent.state) in has_no_start_state_state_types:
            view['is_start_state_checkbutton'].destroy()  # DeciderState is removed if parent change type
        else:
            view['is_start_state_checkbutton'].set_active(bool(self.model.is_start))
            view['is_start_state_checkbutton'].connect('toggled', self.on_toggle_is_start_state)

        if isinstance(self.model.state, DeciderState):
            combo.set_sensitive(False)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))
        pass

    def rename(self):
        self.view['entry_name'].grab_focus()

    def on_toggle_is_start_state(self, button):

        if not self.view['is_start_state_checkbutton'].get_active() == self.model.is_start:
            if self.model.parent is not None:
                try:
                    if self.view['is_start_state_checkbutton'].get_active():
                        self.model.parent.state.start_state_id = self.model.state.state_id
                        logger.debug("New start state '{0}'".format(self.model.state.name))
                    else:
                        self.model.parent.state.start_state_id = None
                        logger.debug("Start state unset, no start state defined")
                except AttributeError as e:
                    logger.warn("Could no change start state: {0}".format(e))
                    # avoid to toggle button
                    self.view['is_start_state_checkbutton'].set_active(bool(self.model.is_start))

    @Model.observe('is_start', assign=True)
    def notify_is_start(self, model, prop_name, info):
        if not self.view['is_start_state_checkbutton'].get_active() == self.model.is_start:
            self.view['is_start_state_checkbutton'].set_active(bool(self.model.is_start))

    @Model.observe('state', after=True)
    def notify_name_change(self, model, prop_name, info):
        if info['method_name'] == 'name':
            self.view['entry_name'].set_text(self.model.state.name)

    def change_name(self, entry, event):
        entry_text = entry.get_text()
        if self.model.state.name != entry_text:
            try:
                self.model.state.name = entry_text
                logger.debug("State '{0}' changed name to '{1}'".format(self.model.state.name, entry_text))
            except (TypeError, ValueError) as e:
                logger.warn("Could not change state name: {0}".format(e))
            self.view['entry_name'].set_text(self.model.state.name)

    def change_type(self, widget, model=None, info=None):
        # TODO this function should be realized by a call of the ContainerState (change_type)
        # for clean use it is a remove and add approach at the moment
        type_text = widget.get_active_text()
        if type_text not in self.state_types_dict:
            logger.error("The desired state type does not exist")
            exit(-1)
        target_class = self.state_types_dict[type_text]['class']
        if target_class != type(self.model.state):
            state_name = self.model.state.name
            logger.debug("Change type of State '{0}' from {1} to {2}".format(state_name,
                                                                             type(self.model.state),
                                                                             target_class))
            if self.model.state.parent is None:
                from awesome_tool.mvc.singleton import state_machine_manager_model
                sm_id = state_machine_manager_model.state_machine_manager.get_sm_id_for_state(self.model.state)
                state_machine = state_machine_manager_model.state_machine_manager.state_machines[sm_id]
                # TODO: refactor
                state_model = state_machine.change_root_state_type(self.model, target_class)
            else:
                # TODO: refactor
                state_model = self.model.parent.state.change_state_type(self.model, target_class)

            self.relieve_model(self.model)
            self.observe_model(state_model)
            self.model = state_model
        else:
            logger.debug("DON'T Change type of State '{0}' from {1} to {2}".format(self.model.state.name,
                                                                             type(self.model.state),
                                                                             target_class))

    def check_for_enter(self, entry, event):
        key_name = keyval_name(event.keyval)
        if key_name in ["Return", "KP_Enter"]:
            self.change_name(entry, None)

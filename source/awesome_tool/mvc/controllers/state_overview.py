import gtk
from mvc.controllers.extended_controller import ExtendedController
from gtkmvc import Controller, Model

from statemachine.states.state import State, StateType
from statemachine.states.container_state import ContainerState
from statemachine.states.concurrency_state import ConcurrencyState

from statemachine.states.execution_state import ExecutionState
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from statemachine.states.library_state import LibraryState

from mvc.models import ContainerStateModel, StateModel

from utils import log
logger = log.get_logger(__name__)


class StateOverviewController(ExtendedController, Model):
    """Controller handling the view of properties/attributes of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.source_editor.SourceEditorView` and the properties of the
    :class:`mvc.models.state.StateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param mvc.models.StateModel model: The state model containing the data
    :param mvc.views.SourceEditorView view: The GTK view showing the data as a table
    """

    # TODO Missing functions

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        # StateType = Enum('STATE_TYPE', 'EXECUTION HIERARCHY BARRIER_CONCURRENCY PREEMPTION_CONCURRENCY LIBRARY')
        self.state_types_dict = {}
        self.state_types_dict[str(StateType.EXECUTION)] = {'Enum': StateType.EXECUTION, 'class': ExecutionState}
        self.state_types_dict[str(StateType.HIERARCHY)] = {'Enum': StateType.HIERARCHY, 'class': HierarchyState}
        self.state_types_dict[str(StateType.BARRIER_CONCURRENCY)] = {'Enum': StateType.BARRIER_CONCURRENCY, 'class': BarrierConcurrencyState}
        self.state_types_dict[str(StateType.PREEMPTION_CONCURRENCY)] = {'Enum': StateType.PREEMPTION_CONCURRENCY, 'class': PreemptiveConcurrencyState}
        # self.state_types_dict[LibraryState] = {'Enum': StateType.LIBRARY}

        view['entry_name'].connect('focus-out-event', self.change_name)
        view['description_textview'].connect('focus-out-event', self.change_description)
        if self.model.state.name:
            view['entry_name'].set_text(self.model.state.name)
        view['label_id_value'].set_text(self.model.state.state_id)
        #view['label_type_value'].set_text(str(self.model.state.state_type))
        view['description_textview'].set_buffer(self.model.state.description)
        view['description_textview'].set_accepts_tab(False)

        l_store = gtk.ListStore(str)
        combo = gtk.ComboBox()
        combo.set_model(l_store)
        cell = gtk.CellRendererText()
        combo.pack_start(cell, True)
        combo.add_attribute(cell, 'text', 0)
        combo.show_all()
        view['type_viewport'].add(combo)
        view['type_viewport'].show()
        l_store.append([str(self.model.state.state_type)])
        for key in self.state_types_dict.keys():
            if not key == str(self.model.state.state_type):
                l_store.append([key])
        combo.set_active(0)
        view['type_combobox'] = combo
        view['type_combobox'].connect('changed', self.change_type)

        view['is_start_state_checkbutton'].set_active(bool(self.model.is_start))
        view['is_start_state_checkbutton'].connect('toggled', self.on_toggle_is_start_state)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    def on_toggle_is_start_state(self, button):

        if not self.view['is_start_state_checkbutton'].get_active() == self.model.is_start:
            if self.view['is_start_state_checkbutton'].get_active():
                logger.debug("set start_state %s" % self.model.state.state_id)
                self.model.parent.state.start_state = self.model.state.state_id
            else:
                logger.debug("set start_state %s" % str(None))
                self.model.parent.state.start_state = None

    @Model.observe('is_start', assign=True)
    def notify_is_start(self, model, prop_name, info):
        if not self.view['is_start_state_checkbutton'].get_active() == self.model.is_start:
            self.view['is_start_state_checkbutton'].set_active(bool(self.model.is_start))

    def change_name(self, entry, otherwidget):
        entry_text = entry.get_text()
        if len(entry_text) > 0 and not self.model.state.name == entry_text:
            logger.debug("State %s changed name from '%s' to: '%s'\n" % (self.model.state.state_id,
                                                                         self.model.state.name, entry_text))
            self.model.state.name = entry_text
            self.view['entry_name'].set_text(self.model.state.name)

    def change_description(self, textview, otherwidget):
        tbuffer = textview.get_buffer()
        entry_text = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())

        if len(entry_text) > 0:
            logger.debug("State %s changed description from '%s' to: '%s'\n" % (self.model.state.state_id,
                                                                                self.model.state.description, entry_text))
            self.model.state.description = entry_text
            self.view['description_textview'].get_buffer().set_text(self.model.state.description)

    def change_type(self, widget, model=None, info=None):
        # TODO this function should be realized by a call of the ContainerState (change_type)
        # for clean use it is a remove and add approach at the moment
        type_text = widget.get_active_text()
        if not type_text == str(self.model.state.state_type) and type_text in self.state_types_dict:
            state_name = self.model.state.name
            state_id = self.model.state.state_id
            logger.debug("change type of State %s from %s to %s" % (state_name, self.model.state.state_type, type_text))

            if not self.model.parent:
                logger.warning("ROOT_STATE types could not be changed!!!")
                return

            new_state = self.state_types_dict[type_text]['class'](state_name, state_id)
            parent_state_model = self.model.parent
            self.relieve_model(self.model)
            parent_state_model.state.remove_state(state_id)
            parent_state_model.state.add_state(new_state)
            state_model = parent_state_model.states[state_id]
            self.observe_model(state_model)
            self.model = state_model

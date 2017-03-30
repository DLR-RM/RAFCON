# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_overview
   :synopsis: A module that holds the controller to give access to basic state information and to edit state name,
     is-start-state flag and state-type.

"""
import glib
import gtk
from gtk.gdk import keyval_name
from gtkmvc import Model

from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState, DeciderState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.states.state import StateType
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.helpers.label import format_cell
from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateOverviewController(ExtendedController, Model):
    """Controller handling the view of properties/attributes of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`gui.views.source_editor.SourceEditorView` and the properties of the
    :class:`gui.models.state.StateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param rafcon.gui.models.StateModel model: The state model containing the data
    :param rafcon.gui.views.SourceEditorView view: The GTK view showing the data as a table
    """

    def __init__(self, model, view, with_is_start_state_check_box=False):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)

        self.state_types_dict = {}
        self.with_is_start_state_check_box = with_is_start_state_check_box

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

        view['entry_name'].connect('focus-out-event', self.on_focus_out)
        view['entry_name'].connect('key-press-event', self.check_for_enter)
        if self.model.state.name:
            view['entry_name'].set_text(self.model.state.name)
        view['label_id_value'].set_text(self.model.state.state_id)

        cell = gtk.CellRendererText()
        format_cell(cell, constants.BUTTON_MIN_HEIGHT, constants.GRID_SIZE)
        l_store = gtk.ListStore(str)
        combo = gtk.ComboBox()
        combo.set_name("state_type_combo")
        combo.set_focus_on_click(False)
        combo.set_model(l_store)
        combo.pack_start(cell, True)
        combo.add_attribute(cell, 'text', 0)
        combo.show_all()
        view['type_viewport'].add(combo)
        view['type_viewport'].show()

        # Prepare LAbel for state_name -> Library states cannot be changed
        if isinstance(self.model.state, LibraryState):
            l_store.prepend(['LIBRARY'])
            combo.set_sensitive(False)

            self.view['library_path'].set_text(self.model.state.library_path + "/" + self.model.state.library_name)
            view['show_content_checkbutton'].set_active(self.model.meta['gui']['show_content'] is True)
            view['show_content_checkbutton'].connect('toggled', self.on_toggle_show_content)
        else:
            self.view['properties_widget'].remove(self.view['label_library_path'])
            self.view['properties_widget'].remove(self.view['library_path'])
            self.view['properties_widget'].remove(self.view['label_show_content'])
            self.view['properties_widget'].remove(self.view['show_content_checkbutton'])
            self.view['properties_widget'].resize(2, 5)

            for key, value in self.state_types_dict.iteritems():
                if isinstance(self.model.state, value['class']):
                    l_store.prepend([key])
                else:
                    l_store.append([key])

        combo.set_active(0)
        view['type_combobox'] = combo
        view['type_combobox'].connect('changed', self.change_type)

        # Prepare "is start state check button"
        has_no_start_state_state_types = [BarrierConcurrencyState, PreemptiveConcurrencyState]
        if not self.with_is_start_state_check_box or isinstance(self.model.state, DeciderState) or \
                self.model.state.is_root_state or type(self.model.parent.state) in has_no_start_state_state_types:
            view['is_start_state_checkbutton'].destroy()
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
        # self.adapt(self.__state_property_adapter("name", "input_name"))
        pass

    def rename(self):
        self.view['entry_name'].grab_focus()

    def on_toggle_is_start_state(self, button):
        if not button.get_active() == self.model.is_start:
            # from rafcon.gui.helpers import state_machine as state_machine
            gui_helper_state_machine.selected_state_toggle_is_start_state()

    def on_toggle_show_content(self, checkbox):
        self.model.meta['gui']['show_content'] = checkbox.get_active()
        msg = MetaSignalMsg(origin='state_overview', change='show_content', affects_children=False)
        self.model.meta_signal.emit(msg)

    @Model.observe('is_start', assign=True)
    def notify_is_start(self, model, prop_name, info):
        if self.view is not None and not self.view['is_start_state_checkbutton'].get_active() == self.model.is_start:
            self.view['is_start_state_checkbutton'].set_active(bool(self.model.is_start))

    @Model.observe('state', after=True)
    def notify_name_change(self, model, prop_name, info):
        if self.view is not None and info['method_name'] == 'name':
            self.view['entry_name'].set_text(self.model.state.name)

    def on_focus_out(self, entry, event):
        # We have to use idle_add to prevent core dumps:
        # https://mail.gnome.org/archives/gtk-perl-list/2005-September/msg00143.html
        glib.idle_add(self.change_name, entry.get_text())

    def change_name(self, new_name):
        if self.model.state.name != new_name:
            try:
                self.model.state.name = new_name
                logger.debug("State '{0}' changed name to '{1}'".format(self.model.state.name, new_name))
            except (TypeError, ValueError) as e:
                logger.warn("Could not change state name: {0}".format(e))
            self.view['entry_name'].set_text(self.model.state.name)

    def change_type(self, widget, model=None, info=None):
        type_text = widget.get_active_text()
        if type_text not in self.state_types_dict:
            logger.error("The desired state type does not exist")
            return

        target_class = self.state_types_dict[type_text]['class']
        if target_class != type(self.model.state):
            state_name = self.model.state.name
            logger.debug("Change type of State '{0}' from {1} to {2}".format(state_name,
                                                                             type(self.model.state).__name__,
                                                                             target_class.__name__))
            try:
                if self.model.state.is_root_state:
                    self.model.state.parent.change_root_state_type(target_class)
                else:
                    self.model.state.parent.change_state_type(self.model.state, target_class)
            except Exception as e:
                logger.error("An error occurred while changing the state type: {0}".format(e))
                raise

        else:
            logger.debug("DON'T Change type of State '{0}' from {1} to {2}".format(self.model.state.name,
                                                                                   type(self.model.state).__name__,
                                                                                   target_class.__name__))

    def check_for_enter(self, entry, event):
        key_name = keyval_name(event.keyval)
        if key_name in ["Return", "KP_Enter"]:
            self.change_name(entry.get_text())

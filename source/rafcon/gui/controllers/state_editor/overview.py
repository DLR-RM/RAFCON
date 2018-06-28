# Copyright (C) 2015-2018 DLR
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
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.states.state import StateType
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.helpers.label import format_cell
from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.models import AbstractStateModel, LibraryStateModel
from rafcon.gui.views.state_editor.overview import StateOverviewView
from rafcon.gui.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateOverviewController(ExtendedController):
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
        assert isinstance(model, AbstractStateModel)
        assert isinstance(view, StateOverviewView)
        super(StateOverviewController, self).__init__(model, view)

        self._external_update = False
        self.state_types_dict = {}
        self.with_is_start_state_check_box = with_is_start_state_check_box

    @staticmethod
    def change_state_type_class_dict(state):
        state_types_dict = {}
        if isinstance(state, DeciderState):
            # logger.info(str(StateType))
            state_types_dict[str(StateType.DECIDER_STATE).split('.')[1]] = {
                'Enum': StateType.DECIDER_STATE, 'class': DeciderState}
        else:
            state_types_dict[str(StateType.EXECUTION).split('.')[1]] = {
                'Enum': StateType.EXECUTION, 'class': ExecutionState}
            state_types_dict[str(StateType.HIERARCHY).split('.')[1]] = {
                'Enum': StateType.HIERARCHY, 'class': HierarchyState}
            state_types_dict[str(StateType.BARRIER_CONCURRENCY).split('.')[1]] = {
                'Enum': StateType.BARRIER_CONCURRENCY, 'class': BarrierConcurrencyState}
            state_types_dict[str(StateType.PREEMPTION_CONCURRENCY).split('.')[1]] = {
                'Enum': StateType.PREEMPTION_CONCURRENCY, 'class': PreemptiveConcurrencyState}
        return state_types_dict

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application

        :param rafcon.gui.views.state_editor.overview.StateOverviewView view: A state overview view instance
        """
        # prepare State Type Change ComboBox
        super(StateOverviewController, self).register_view(view)
        self.state_types_dict = self.change_state_type_class_dict(self.model.state)

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

        # Prepare label for state_name -> Library states cannot be changed
        if isinstance(self.model, LibraryStateModel):
            l_store.prepend(['LIBRARY'])
            combo.set_sensitive(False)

            self.view['library_path'].set_text(self.model.state.library_path + "/" + self.model.state.library_name)
            self.view['library_path'].set_sensitive(True)
            self.view['library_path'].set_editable(False)
            view['show_content_checkbutton'].set_active(self.model.meta['gui']['show_content'] is True)
            view['show_content_checkbutton'].connect('toggled', self.on_toggle_show_content)
            # self.view['properties_widget'].remove(self.view['show_content_checkbutton'])
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

        # in case the state is inside of a library
        if self.model.state.get_library_root_state():
            view['entry_name'].set_editable(False)
            combo.set_sensitive(False)
            view['is_start_state_checkbutton'].set_sensitive(False)
            if isinstance(self.model, LibraryStateModel):
                self.view['show_content_checkbutton'].set_sensitive(False)

    def rename(self):
        self.view['entry_name'].grab_focus()

    def on_toggle_is_start_state(self, button):
        if not button.get_active() == self.model.is_start:
            # from rafcon.gui.helpers import state_machine as state_machine
            gui_helper_state_machine.selected_state_toggle_is_start_state()

    def on_toggle_show_content(self, checkbox):
        if self._external_update:
            return
        self.model.meta['gui']['show_content'] = checkbox.get_active()
        msg = MetaSignalMsg(origin='state_overview', change='show_content', affects_children=False)
        self.model.meta_signal.emit(msg)

    @ExtendedController.observe("meta_signal", signal=True)
    def show_content_changed(self, model, prop_name, info):
        meta_signal_message = info['arg']
        if meta_signal_message.change == 'show_content':
            self._external_update = True
            self.view['show_content_checkbutton'].set_active(model.meta['gui']['show_content'])
            self._external_update = False

    @ExtendedController.observe('is_start', assign=True)
    def notify_is_start(self, model, prop_name, info):
        if self.view is not None and not self.view['is_start_state_checkbutton'].get_active() == self.model.is_start:
            self.view['is_start_state_checkbutton'].set_active(bool(self.model.is_start))

    @ExtendedController.observe('state', after=True)
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
        gui_helper_state_machine.change_state_type_with_error_handling_and_logger_messages(self.model, target_class)

    def check_for_enter(self, entry, event):
        key_name = keyval_name(event.keyval)
        if key_name in ["Return", "KP_Enter"]:
            self.change_name(entry.get_text())

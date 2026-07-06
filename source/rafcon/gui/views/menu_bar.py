# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import Gio, GLib, Gtk
from rafcon.design_patterns.mvc.view import View

from rafcon.gui import glade
from rafcon.gui.config import global_gui_config


class MenuActionProxy(object):
    """Widget-like facade for a menu entry backed by a Gio action

    GTK4 menu entries are not widgets. This proxy emulates the small part of the former
    Gtk.(Check)MenuItem API used by the controllers (get_active/set_active/set_sensitive/
    hide/show) on top of the backing Gio.SimpleAction, so call sites stay unchanged.
    """

    def __init__(self, view, name):
        self._view = view
        self._name = name

    @property
    def _action(self):
        container = self._view.action_container or Gio.Application.get_default()
        return container.lookup_action(self._name) if container else None

    def get_active(self):
        action = self._action
        state = action.get_state() if action else None
        return bool(state.get_boolean()) if state is not None else False

    def set_active(self, active):
        action = self._action
        if action is not None:
            action.change_state(GLib.Variant.new_boolean(bool(active)))

    def set_sensitive(self, sensitive):
        action = self._action
        if action is not None:
            action.set_enabled(sensitive)

    def hide(self):
        # menu entries of a GMenu model cannot be hidden; disabling is the closest equivalent
        self.set_sensitive(False)

    def show(self):
        self.set_sensitive(True)


class MenuBarView(View):
    """Menu bar of the main window.

    GTK4 removed GtkMenuBar/GtkMenu/GtkMenuItem widgets. The menu bar is now a GMenu model
    (see menu_bar.ui) rendered by a GtkPopoverMenuBar. Menu entries are no longer widgets:
    each entry triggers the application action "app.<name>", where <name> is the former
    GTK3 menu item widget id. Activation handlers, sensitivity (Gio.SimpleAction.set_enabled)
    and check states (stateful actions) are wired on the Gtk.Application by the controller.
    """

    # former GtkCheckMenuItems: their actions must be created stateful (boolean)
    toggle_actions = ['grid', 'data_flow_mode', 'show_data_flows', 'show_transitions',
                      'show_data_values', 'show_aborted_preempted', 'full_screen']

    actions = [
        # File
        'new', 'open', 'save', 'save_as', 'save_as_copy', 'save_state_as', 'menu_preferences',
        'refresh_all', 'refresh_libraries', 'bake_state_machine', 'layout_state_machine', 'quit',
        # Edit
        'cut', 'copy', 'paste', 'is_start_state', 'add', 'group', 'ungroup', 'substitute_state',
        'delete', 'undo', 'redo', 'search',
        # View
        'expert_view',
        # Execution
        'start', 'start_from_selected', 'run_to_selected', 'run_selected', 'only_run_selected',
        'pause', 'stop', 'step_mode', 'step_into', 'step_over', 'step_out', 'backward_step',
        # Help
        'about',
    ] + toggle_actions

    def __init__(self):
        super().__init__(builder_filename=glade.get_glade_path('menu_bar.ui'), parent='menubar')

        # the Gio.ActionMap holding the menu actions (Gtk.Application or a Gio.SimpleActionGroup);
        # set by the menu bar controller when the actions are created
        self.action_container = None

        # menu section the controller fills with the recently opened state machines
        self.sub_menu_open_recently = self['open_recent_section']

        # accelerator per action, applied by the controller via Gtk.Application.set_accels_for_action
        self.accelerators = {'new': '<control>N',
                             'open': '<control>O',
                             'save': '<control>S',
                             'quit': '<control>Q',
                             'cut': '<control>X',
                             'copy': '<control>C',
                             'paste': '<control>V',
                             }
        shortcuts = global_gui_config.get_config_value('SHORTCUTS')
        for action_name in self.actions:
            if action_name in shortcuts and shortcuts[action_name]:
                action_shortcuts = shortcuts[action_name]
                main_shortcut = action_shortcuts[0] if isinstance(action_shortcuts, list) else action_shortcuts
                self.set_menu_item_accelerator(action_name, main_shortcut)

    def __getitem__(self, key):
        # menu entries are not widgets anymore; hand out an action proxy for them
        if key in self.actions:
            return MenuActionProxy(self, key)
        return super().__getitem__(key)

    def set_menu_item_icon(self, menu_item_name, uni_code=None):
        """No-op: GMenu entries are not widgets, so the FontAwesome icon boxes of GTK3 are gone"""
        pass

    def set_menu_item_sensitive(self, menu_item_name, sensitive):
        """Enable/disable the application action backing the menu entry"""
        container = self.action_container or Gio.Application.get_default()
        action = container.lookup_action(menu_item_name) if container else None
        if action is not None:
            action.set_enabled(sensitive)

    def set_menu_item_accelerator(self, menu_item_name, accel_code, remove_old=False):
        """Record the accelerator for an action; the controller registers it on the application"""
        key, mod = Gtk.accelerator_parse(accel_code)[1:]
        if not key:
            return
        self.accelerators[menu_item_name] = accel_code
        app = Gio.Application.get_default()
        container = self.action_container or app
        if isinstance(app, Gtk.Application) and container and container.lookup_action(menu_item_name):
            app.set_accels_for_action("app.{}".format(menu_item_name), [accel_code])

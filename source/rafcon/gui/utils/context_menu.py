# Copyright (C) 2026 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html

"""
.. module:: context_menu
   :synopsis: GTK4 replacement for the GTK3 Gtk.Menu based context menus

GTK4 removed Gtk.Menu/Gtk.MenuItem. This module provides a small compatibility layer that
keeps the imperative menu-building style of the former controllers (append items with
callbacks, separators and nested sub menus) and renders the menu as a Gtk.PopoverMenu
backed by a generated Gio.Menu model and Gio.SimpleActionGroup.
"""

from gi.repository import Gdk, Gio, GLib, Gtk

from rafcon.utils import log
logger = log.get_logger(__name__)


class ContextMenuItem(object):
    """Description of one context menu entry

    Mimics the parts of the former Gtk.MenuItem API used by the controllers (set_sensitive).
    The backing Gio action is created when the menu is popped up.
    """

    def __init__(self, label, callback=None, callback_args=(), checked=None):
        self.label = label
        self.callback = callback
        self.callback_args = tuple(callback_args)
        # None for plain items; True/False for check items
        self.checked = checked
        self.sensitive = True

    def set_sensitive(self, sensitive):
        self.sensitive = bool(sensitive)

    def get_active(self):
        """State of a check item (GTK3 Gtk.CheckMenuItem API)"""
        return bool(self.checked)

    def set_active(self, active):
        self.checked = bool(active)


class ContextMenu(object):
    """Imperatively built context menu, shown as Gtk.PopoverMenu

    Replacement for the GTK3 ``Gtk.Menu`` in the right-click menus. Entries are appended as
    :class:`ContextMenuItem` (see the helper functions in :mod:`rafcon.gui.helpers.label`),
    separators split the menu into sections and sub menus nest further menus.
    """

    ACTION_GROUP_PREFIX = 'ctxmenu'

    def __init__(self):
        # entries: ('item', ContextMenuItem) | ('separator',) | ('submenu', label, ContextMenu)
        self._entries = []
        self._popover = None

    def append(self, item):
        if not isinstance(item, ContextMenuItem):
            raise TypeError("Context menus only accept ContextMenuItem entries, got {0}".format(type(item)))
        self._entries.append(('item', item))
        return item

    def append_separator(self):
        self._entries.append(('separator',))

    def add_submenu(self, label):
        """Appends a sub menu entry and returns the nested ContextMenu"""
        sub_menu = ContextMenu()
        self._entries.append(('submenu', label, sub_menu))
        return sub_menu

    def show_all(self):
        """No-op for GTK3 call-site compatibility (popovers show on popup)"""
        pass

    def is_empty(self):
        return not self._entries

    def _build_model(self, action_group, name_counter):
        """Creates the Gio.Menu model and fills the action group"""
        menu_model = Gio.Menu()
        section = Gio.Menu()
        menu_model.append_section(None, section)
        for entry in self._entries:
            if entry[0] == 'separator':
                if section.get_n_items():
                    section = Gio.Menu()
                    menu_model.append_section(None, section)
            elif entry[0] == 'submenu':
                _, label, sub_menu = entry
                section.append_submenu(label, sub_menu._build_model(action_group, name_counter))
            else:
                item = entry[1]
                action_name = 'item{0}'.format(name_counter[0])
                name_counter[0] += 1
                if item.checked is None:
                    action = Gio.SimpleAction.new(action_name, None)
                else:
                    action = Gio.SimpleAction.new_stateful(action_name, None,
                                                           GLib.Variant.new_boolean(item.checked))

                def on_activate(action, parameter, item=item):
                    if item.checked is not None:
                        item.checked = not item.checked
                        action.set_state(GLib.Variant.new_boolean(item.checked))
                    if item.callback is not None:
                        item.callback(item, *item.callback_args)

                action.connect('activate', on_activate)
                action.set_enabled(item.sensitive)
                action_group.add_action(action)
                section.append(item.label, '{0}.{1}'.format(self.ACTION_GROUP_PREFIX, action_name))
        return menu_model

    def popup_at(self, widget, x, y):
        """Pops up the menu as popover on the given widget at widget coordinates (x, y)"""
        if self.is_empty():
            return
        action_group = Gio.SimpleActionGroup()
        menu_model = self._build_model(action_group, [0])
        popover = Gtk.PopoverMenu.new_from_model(menu_model)
        popover.insert_action_group(self.ACTION_GROUP_PREFIX, action_group)
        popover.set_parent(widget)
        rect = Gdk.Rectangle()
        rect.x, rect.y, rect.width, rect.height = int(x), int(y), 1, 1
        popover.set_pointing_to(rect)
        popover.set_has_arrow(False)
        popover.connect('closed', self._on_popover_closed)
        self._popover = popover
        popover.popup()

    def popdown(self):
        if self._popover is not None:
            self._popover.popdown()

    def _on_popover_closed(self, popover):
        # unparent deferred: destroying the popover from within its own 'closed' handler warns
        def unparent():
            popover.unparent()
            return False
        GLib.idle_add(unparent)
        if self._popover is popover:
            self._popover = None

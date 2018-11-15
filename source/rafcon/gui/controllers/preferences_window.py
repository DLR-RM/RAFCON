# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: preferences_window
   :synopsis: a module holding the controller for the configuration of GUI and Core

"""
from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import GObject
from future.utils import string_types
from builtins import str
import yaml_configuration.config
from os.path import dirname

from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.helpers.label import react_to_event
from rafcon.gui.models.config_model import ConfigModel
from rafcon.gui.views.preferences_window import PreferencesWindowView
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.utils import log

logger = log.get_logger(__name__)


class PreferencesWindowController(ExtendedController):
    """Controller handling the configuration GUI
    """

    KEY_STORAGE_ID = 0
    VALUE_STORAGE_ID = 1
    TEXT_VISIBLE_STORAGE_ID = 2
    TOGGLE_ACTIVATABLE_STORAGE_ID = 3
    TOGGLE_VISIBLE_STORAGE_ID = 4
    TEXT_EDITABLE_STORAGE_ID = 5
    TOGGLE_VALUE_STORAGE_ID = 6

    def __init__(self, core_config_model, view, gui_config_model):
        assert isinstance(view, PreferencesWindowView)
        assert isinstance(core_config_model, ConfigModel)
        assert isinstance(gui_config_model, ConfigModel)
        ExtendedController.__init__(self, core_config_model, view)
        self.core_config_model = core_config_model
        self.gui_config_model = gui_config_model
        self.observe_model(gui_config_model)

        # (config_key, config_value, text_visible, toggle_activatable, toggle_visible, text_editable, toggle_value)
        self.core_list_store = Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING, bool, bool, bool, bool, bool)
        self.library_list_store = Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING)
        self.gui_list_store = Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING, bool, bool, bool, bool, bool)
        self.shortcut_list_store = Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING)

        self._lib_counter = 0
        self._gui_checkbox = Gtk.CheckButton(label="GUI Config")
        self._core_checkbox = Gtk.CheckButton(label="Core Config")
        self._last_path = self.core_config_model.config.path

    def __destroy(self):
        """Remove controller from parent controller and/or destroy it self."""
        if self.parent:
            self.parent.remove_controller(self)
        else:
            self.destroy()

    def register_view(self, view):
        """Called when the View was registered"""
        super(PreferencesWindowController, self).register_view(view)
        self.view['add_library_button'].connect('clicked', self._on_add_library)
        self.view["remove_library_button"].connect('clicked', self._on_remove_library)

        self.view['config_tree_view'].connect("button_press_event", self._on_row_clicked_trigger_toggle_of_boolean,
                                              self.core_config_model, self.core_list_store)
        self.view['gui_tree_view'].connect("button_press_event", self._on_row_clicked_trigger_toggle_of_boolean,
                                           self.gui_config_model, self.gui_list_store)

        self.view['core_config_value_renderer'].set_property('editable', True)
        self.view['core_config_value_renderer'].connect('edited', self._on_config_value_changed, self.core_config_model,
                                                        self.core_list_store)

        self.view['gui_config_value_renderer'].set_property('editable', True)
        self.view['gui_config_value_renderer'].connect('edited', self._on_config_value_changed, self.gui_config_model,
                                                       self.gui_list_store)

        self.view['shortcut_config_value_renderer'].set_property('editable', True)
        self.view['shortcut_config_value_renderer'].connect('edited', self._on_shortcut_changed)

        self.view['library_config_key_renderer'].set_property('editable', True)
        self.view['library_config_key_renderer'].connect('edited', self._on_library_name_changed)

        self.view['library_config_value_renderer'].set_property('editable', True)
        self.view['library_config_value_renderer'].connect('edited', self._on_library_path_changed)

        self.view['config_tree_view'].set_model(self.core_list_store)
        self.view['library_tree_view'].set_model(self.library_list_store)
        self.view['gui_tree_view'].set_model(self.gui_list_store)
        self.view['shortcut_tree_view'].set_model(self.shortcut_list_store)

        self.view['apply_button'].connect("clicked", self._on_apply_button_clicked)
        self.view['ok_button'].connect('clicked', self._on_ok_button_clicked)
        self.view['cancel_button'].connect('clicked', self._on_cancel_button_clicked)

        self.view['import_button'].connect("clicked", self._on_import_config)
        self.view['export_button'].connect('clicked', self._on_export_config)

        self.view['preferences_window'].connect('delete_event', self._on_delete_event)

        self.update_all()

    @ExtendedController.observe('config', after=True)
    def on_config_value_changed(self, config_m, prop_name, info):
        """Callback when a config value has been changed

        Only collects information, delegates handling further to _handle_config_update

        :param ConfigModel config_m: The config model that has been changed
        :param str prop_name: Should always be 'config'
        :param dict info: Information e.g. about the changed config key
        """
        config_key = info['args'][1] if "key" not in info['kwargs'] else info['kwargs']['key']
        # config_value = info['args'][-1] if "value" not in info['kwargs'] else info['kwargs']['value']
        self._handle_config_update(config_m, config_key)

    @ExtendedController.observe('preliminary_config', after=True)
    def on_preliminary_config_changed(self, config_m, prop_name, info):
        """Callback when a preliminary config value has been changed

        Mainly collects information, delegates handling further to _handle_config_update

        :param ConfigModel config_m: The config model that has been changed
        :param str prop_name: Should always be 'preliminary_config'
        :param dict info: Information e.g. about the changed config key
        """
        self.check_for_preliminary_config()

        method_name = info['method_name']  # __setitem__, __delitem__, clear, ...

        if method_name in ['__setitem__', '__delitem__']:
            config_key = info['args'][0]
            self._handle_config_update(config_m, config_key)
        # Probably the preliminary config has been cleared, update corresponding list stores
        elif config_m is self.core_config_model:
            self.update_core_config_list_store()
            self.update_libraries_list_store()
        else:
            self.update_gui_config_list_store()
            self.update_shortcut_settings()

    def _handle_config_update(self, config_m, config_key):
        """Handles changes in config values

        The method ensure that the correct list stores are updated with the new values.


        :param ConfigModel config_m: The config model that has been changed
        :param config_key: The config key who's value has been changed
        :return:
        """
        if config_key == "LIBRARY_PATHS":
            self.update_libraries_list_store()
        if config_key == "SHORTCUTS":
            self.update_shortcut_settings()
        else:
            self.update_config_value(config_m, config_key)

    def update_all(self):
        """Shorthand method to update all collection information
        """
        self.update_path_labels()
        self.update_core_config_list_store()
        self.update_gui_config_list_store()
        self.update_libraries_list_store()
        self.update_shortcut_settings()
        self.check_for_preliminary_config()

    def check_for_preliminary_config(self):
        """Activates the 'Apply' button if there are preliminary changes
        """
        if any([self.model.preliminary_config, self.gui_config_model.preliminary_config]):
            self.view['apply_button'].set_sensitive(True)
        else:
            self.view['apply_button'].set_sensitive(False)

    def update_path_labels(self):
        """Update labels showing config paths
        """
        self.view['core_label'].set_text("Core Config Path: " + str(self.core_config_model.config.config_file_path))
        self.view['gui_label'].set_text("GUI Config Path: " + str(self.gui_config_model.config.config_file_path))

    def update_config_value(self, config_m, config_key):
        """Updates the corresponding list store of a changed config value

        :param ConfigModel config_m: The config model that has been changed
        :param str config_key: The config key who's value has been changed
        """
        config_value = config_m.get_current_config_value(config_key)
        if config_m is self.core_config_model:
            list_store = self.core_list_store
        elif config_m is self.gui_config_model:
            list_store = self.gui_list_store
        else:
            return
        self._update_list_store_entry(list_store, config_key, config_value)

    def _update_list_store_entry(self, list_store, config_key, config_value):
        """Helper method to update a list store

        :param Gtk.ListStore list_store: List store to be updated
        :param str config_key: Config key to search for
        :param config_value: New config value
        :returns: Row of list store that has been updated
        :rtype: int
        """
        for row_num, row in enumerate(list_store):
            if row[self.KEY_STORAGE_ID] == config_key:
                row[self.VALUE_STORAGE_ID] = str(config_value)
                row[self.TOGGLE_VALUE_STORAGE_ID] = config_value
                return row_num

    @staticmethod
    def _update_list_store(config_m, list_store, ignore_keys=None):
        """Generic method to create list store for a given config model

        :param ConfigModel config_m: Config model to read into list store
        :param Gtk.ListStore list_store: List store to be filled
        :param list ignore_keys: List of keys that should be ignored
        """
        ignore_keys = [] if ignore_keys is None else ignore_keys
        list_store.clear()
        for config_key in sorted(config_m.config.keys):
            if config_key in ignore_keys:
                continue
            config_value = config_m.get_current_config_value(config_key)
            # (config_key, text, text_visible, toggle_activatable, toggle_visible, text_editable, toggle_state)
            if isinstance(config_value, bool):
                list_store.append((str(config_key), str(config_value), False, True, True, False, config_value))
            else:
                list_store.append((str(config_key), str(config_value), True, False, False, True, config_value))

    def update_core_config_list_store(self):
        """Create list store for the core configuration
        """
        self._update_list_store(self.core_config_model, self.core_list_store, ignore_keys=['TYPE', 'LIBRARY_PATHS'])

    def update_gui_config_list_store(self):
        """Create list store for the GUI configuration
        """
        self._update_list_store(self.gui_config_model, self.gui_list_store, ignore_keys=['TYPE', 'SHORTCUTS'])

    def update_libraries_list_store(self):
        """Creates the list store for the libraries
        """
        self.library_list_store.clear()
        libraries = self.core_config_model.get_current_config_value("LIBRARY_PATHS", use_preliminary=True, default={})
        library_names = sorted(libraries.keys())
        for library_name in library_names:
            library_path = libraries[library_name]
            self.library_list_store.append((library_name, library_path))

    def update_shortcut_settings(self):
        """Creates the list store for the shortcuts
        """
        self.shortcut_list_store.clear()
        shortcuts = self.gui_config_model.get_current_config_value("SHORTCUTS", use_preliminary=True, default={})
        actions = sorted(shortcuts.keys())
        for action in actions:
            keys = shortcuts[action]
            self.shortcut_list_store.append((str(action), str(keys)))

    @staticmethod
    def _select_row_by_column_value(tree_view, list_store, column, value):
        """Helper method to select a tree view row

        :param Gtk.TreeView tree_view: Tree view who's row is to be selected
        :param Gtk.ListStore list_store: List store of the tree view
        :param int column: Column in which the value is searched
        :param value: Value to search for
        :returns: Row of list store that has selected
        :rtype: int
        """
        for row_num, iter_elem in enumerate(list_store):
            if iter_elem[column] == value:
                tree_view.set_cursor(row_num)
                return row_num

    def _on_add_library(self, *event):
        """Callback method handling the addition of a new library
        """
        self.view['library_tree_view'].grab_focus()
        if react_to_event(self.view, self.view['library_tree_view'], event):
            temp_library_name = "<LIB_NAME_%s>" % self._lib_counter
            self._lib_counter += 1
            library_config = self.core_config_model.get_current_config_value("LIBRARY_PATHS", use_preliminary=True,
                                                                             default={})
            library_config[temp_library_name] = "<LIB_PATH>"
            self.core_config_model.set_preliminary_config_value("LIBRARY_PATHS", library_config)
            self._select_row_by_column_value(self.view['library_tree_view'], self.library_list_store,
                                             self.KEY_STORAGE_ID, temp_library_name)
            return True

    def _on_remove_library(self, *event):
        """Callback method handling the removal of an existing library
        """
        self.view['library_tree_view'].grab_focus()
        if react_to_event(self.view, self.view['library_tree_view'], event):
            path = self.view["library_tree_view"].get_cursor()[0]
            if path is not None:
                library_name = self.library_list_store[int(path[0])][0]
                library_config = self.core_config_model.get_current_config_value("LIBRARY_PATHS", use_preliminary=True,
                                                                                 default={})
                del library_config[library_name]
                self.core_config_model.set_preliminary_config_value("LIBRARY_PATHS", library_config)
                if len(self.library_list_store) > 0:
                    self.view['library_tree_view'].set_cursor(min(path[0], len(self.library_list_store) - 1))
            return True

    def _on_checkbox_toggled(self, renderer, path, config_m, config_list_store):
        """Callback method handling a config toggle event

        :param Gtk.CellRenderer renderer: Cell renderer that has been toggled
        :param path: Path within the list store
        :param ConfigModel config_m: The config model related to the toggle option
        :param Gtk.ListStore config_list_store: The list store related to the toggle option
        """
        config_key = config_list_store[int(path)][self.KEY_STORAGE_ID]
        config_value = bool(config_list_store[int(path)][self.TOGGLE_VALUE_STORAGE_ID])
        config_value ^= True
        config_m.set_preliminary_config_value(config_key, config_value)

    def _on_row_clicked_trigger_toggle_of_boolean(self, widget, event, config_model, list_store):
        # click with left mouse button
        if event.type == Gdk.EventType.BUTTON_PRESS and event.get_button()[1] == 1:
            x = int(event.x)
            y = int(event.y)
            pthinfo = widget.get_path_at_pos(x, y)
            if pthinfo is not None:
                path, col, _, _ = pthinfo
                widget.grab_focus()
                widget.set_cursor(path, col, 0)
                if list_store[path][self.TOGGLE_VISIBLE_STORAGE_ID]:
                    self._on_checkbox_toggled(None, path[0], config_model, list_store)

    def _on_import_config(self, *args):
        """Callback method the the import button was clicked

        Shows a dialog allowing to import an existing configuration file
        """
        def handle_import(dialog_text, path_name):
            chooser = Gtk.FileChooserDialog(dialog_text, None,
                                            Gtk.FileChooserAction.SAVE,
                                            (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
                                             Gtk.STOCK_OPEN, Gtk.ResponseType.ACCEPT))
            chooser.set_current_folder(path_name)
            response = chooser.run()
            if response == Gtk.ResponseType.ACCEPT:
                # get_filename() returns the whole file path inclusively the filename
                config_file = chooser.get_filename()
                config_path = dirname(config_file)
                self._last_path = config_path
                config_dict = yaml_configuration.config.load_dict_from_yaml(config_file)
                config_type = config_dict.get("TYPE")
                if config_type == "SM_CONFIG":
                    del config_dict["TYPE"]
                    self.core_config_model.update_config(config_dict, config_file)
                    logger.info("Imported Core Config from {0}".format(config_file))
                elif config_type == "GUI_CONFIG":
                    del config_dict["TYPE"]
                    self.gui_config_model.update_config(config_dict, config_file)
                    logger.info("Imported GUI Config from {0}".format(config_file))
                else:
                    logger.error("{0} is not a valid config file".format(config_file))
            elif response == Gtk.ResponseType.CANCEL:
                logger.info("Import of configuration cancelled")
            chooser.destroy()

        handle_import("Import Config Config from", self._last_path)

        self.check_for_preliminary_config()
        self.update_path_labels()

    def _on_export_config(self, *args):
        """Callback method the the export button was clicked

        Shows dialogs allowing to export the configurations into separate files
        """
        response = self._config_chooser_dialog("Export configuration",
                                               "Please select the configuration file(s) to be exported:")
        if response == Gtk.ResponseType.REJECT:
            return

        def handle_export(dialog_text, path, config_m):
            chooser = Gtk.FileChooserDialog(dialog_text, None,
                                            Gtk.FileChooserAction.SAVE,
                                            (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
                                             Gtk.STOCK_SAVE_AS, Gtk.ResponseType.ACCEPT))
            chooser.set_current_folder(path)
            response = chooser.run()
            if response == Gtk.ResponseType.ACCEPT:
                config_file = chooser.get_filename()
                if not config_file:
                    logger.error("Configuration could not be exported! Invalid file name!")
                else:
                    if ".yaml" not in config_file:
                        config_file += ".yaml"
                    if config_m.preliminary_config:
                        logger.warning("There are changes in the configuration that have not yet been applied. These "
                                    "changes will not be exported.")
                    self._last_path = dirname(config_file)
                    config_dict = config_m.as_dict()
                    config_copy = yaml_configuration.config.DefaultConfig(str(config_dict))
                    config_copy.config_file_path = config_file
                    config_copy.path = self._last_path
                    try:
                        config_copy.save_configuration()
                        logger.info("Configuration exported to {}" .format(config_file))
                    except IOError:
                        logger.error("Cannot open file '{}' for writing".format(config_file))
            elif response == Gtk.ResponseType.CANCEL:
                logger.warning("Export Config canceled!")
            chooser.destroy()

        if self._core_checkbox.get_active():
            handle_export("Select file for core configuration", self._last_path, self.core_config_model)

        if self._gui_checkbox.get_active():
            handle_export("Select file for GUI configuration.", self._last_path, self.gui_config_model)

    def _on_ok_button_clicked(self, *args):
        """OK button clicked: Applies the configurations and closes the window
        """
        if self.core_config_model.preliminary_config or self.gui_config_model.preliminary_config:
            self._on_apply_button_clicked()
        self.__destroy()

    def _on_cancel_button_clicked(self, *args):
        """Cancel button clicked: Dismiss preliminary config and close the window
        """
        self.core_config_model.preliminary_config.clear()
        self.gui_config_model.preliminary_config.clear()
        self.__destroy()

    def _on_apply_button_clicked(self, *args):
        """Apply button clicked: Apply the configuration
        """
        refresh_required = self.core_config_model.apply_preliminary_config()
        refresh_required |= self.gui_config_model.apply_preliminary_config()

        if not self.gui_config_model.config.get_config_value("SESSION_RESTORE_ENABLED"):
            import rafcon.gui.backup.session as backup_session
            logger.info("Removing current session")
            backup_session.reset_session()
        if refresh_required:
            from rafcon.gui.singleton import main_window_controller
            main_window_controller.get_controller('menu_bar_controller').on_refresh_all_activate(None, None)
        self._popup_message()

    def _popup_message(self):
        changes_str = ''
        if self.core_config_model.changed_keys_requiring_restart or \
                self.gui_config_model.changed_keys_requiring_restart:
            message = Gtk.MessageDialog(parent=self.view["preferences_window"], flags=Gtk.DialogFlags.MODAL,
                                        type=Gtk.MessageType.INFO, buttons=Gtk.ButtonsType.OK)
            message_string = "You must restart RAFCON to apply following changes: \n"
            for key in (self.core_config_model.changed_keys_requiring_restart |
                        self.gui_config_model.changed_keys_requiring_restart):
                changes_str = ("%s\n%s" % (changes_str, key))
            message.set_markup("%s %s" % (message_string, changes_str))
            message.run()
            message.destroy()

    @staticmethod
    def _on_delete_event(window, event):
        """Called when the window gets destroyed

        Hides the window.

        :param Gtk.Window window: The window
        :param Gdk.Event event: Event data
        """
        window.hide()
        return True

    def _on_library_name_changed(self, renderer, path, new_library_name):
        """Callback handling a change of a library name

        :param Gtk.CellRenderer renderer: Cell renderer showing the library name
        :param path: Path of library within the list store
        :param str new_library_name: New library name
        """
        old_library_name = self.library_list_store[int(path)][self.KEY_STORAGE_ID]
        if old_library_name == new_library_name:
            return
        library_path = self.library_list_store[int(path)][self.VALUE_STORAGE_ID]

        library_config = self.core_config_model.get_current_config_value("LIBRARY_PATHS", use_preliminary=True,
                                                                         default={})
        del library_config[old_library_name]
        library_config[new_library_name] = library_path
        self.core_config_model.set_preliminary_config_value("LIBRARY_PATHS", library_config)
        self._select_row_by_column_value(self.view['library_tree_view'], self.library_list_store,
                                         self.KEY_STORAGE_ID, new_library_name)

    def _on_library_path_changed(self, renderer, path, new_library_path):
        """Callback handling a change of a library path

        :param Gtk.CellRenderer renderer: Cell renderer showing the library path
        :param path: Path of library within the list store
        :param str new_library_path: New library path
        """
        library_name = self.library_list_store[int(path)][self.KEY_STORAGE_ID]

        library_config = self.core_config_model.get_current_config_value("LIBRARY_PATHS", use_preliminary=True,
                                                                         default={})
        library_config[library_name] = new_library_path
        self.core_config_model.set_preliminary_config_value("LIBRARY_PATHS", library_config)
        self._select_row_by_column_value(self.view['library_tree_view'], self.library_list_store,
                                         self.KEY_STORAGE_ID, library_name)

    def _on_shortcut_changed(self, renderer, path, new_shortcuts):
        """Callback handling a change of a shortcut

        :param Gtk.CellRenderer renderer: Cell renderer showing the shortcut
        :param path: Path of shortcuts within the list store
        :param str new_shortcuts: New shortcuts
        """
        action = self.shortcut_list_store[int(path)][self.KEY_STORAGE_ID]
        old_shortcuts = self.gui_config_model.get_current_config_value("SHORTCUTS", use_preliminary=True)[action]

        from ast import literal_eval
        try:
            new_shortcuts = literal_eval(new_shortcuts)
            if not isinstance(new_shortcuts, list) and \
               not all([isinstance(shortcut, string_types) for shortcut in new_shortcuts]):
                raise ValueError()
        except (ValueError, SyntaxError):
            logger.warning("Shortcuts must be a list of strings")
            new_shortcuts = old_shortcuts

        shortcuts = self.gui_config_model.get_current_config_value("SHORTCUTS", use_preliminary=True,  default={})
        shortcuts[action] = new_shortcuts
        self.gui_config_model.set_preliminary_config_value("SHORTCUTS", shortcuts)
        self._select_row_by_column_value(self.view['shortcut_tree_view'], self.shortcut_list_store,
                                         self.KEY_STORAGE_ID, action)

    def _on_config_value_changed(self, renderer, path, new_value, config_m, list_store):
        """Callback handling a change of a config value

        :param Gtk.CellRenderer renderer: Cell renderer showing the shortcut
        :param path: Path of shortcuts within the list store
        :param ConfigModel config_m: The config model that is to be changed
        :param Gtk.ListStore list_store: The list store that is to be changed
        """
        config_key = list_store[int(path)][self.KEY_STORAGE_ID]
        old_value = config_m.get_current_config_value(config_key, use_preliminary=True)

        if old_value == new_value:
            return

        # Try to maintain the correct data type, which is extracted from the old value
        if isinstance(old_value, bool):
            if new_value in ["True", "true"]:
                new_value = True
            elif new_value in ["False", "false"]:
                new_value = False
            else:
                logger.warning("'{}' must be a boolean value".format(config_key))
                new_value = old_value
        elif isinstance(old_value, (int, float)):
            try:
                new_value = int(new_value)
            except ValueError:
                try:
                    new_value = float(new_value)
                except ValueError:
                    logger.warning("'{}' must be a numeric value".format(config_key))
                    new_value = old_value

        config_m.set_preliminary_config_value(config_key, new_value)

    def _config_chooser_dialog(self, title_text, description):
        """Dialog to select which config shall be exported

        :param title_text: Title text
        :param description: Description
        """
        dialog = Gtk.Dialog(title_text, self.view["preferences_window"],
                            flags=0, buttons=
                            (Gtk.STOCK_CANCEL, Gtk.ResponseType.REJECT,
                             Gtk.STOCK_OK, Gtk.ResponseType.ACCEPT))
        label = Gtk.Label(label=description)
        label.set_padding(xpad=10, ypad=10)
        dialog.vbox.pack_start(label, True, True, 0)
        label.show()
        self._gui_checkbox = Gtk.CheckButton(label="GUI Config")
        dialog.vbox.pack_start(self._gui_checkbox, True, True, 0)
        self._gui_checkbox.show()
        self._core_checkbox = Gtk.CheckButton(label="Core Config")
        self._core_checkbox.show()
        dialog.vbox.pack_start(self._core_checkbox, True, True, 0)
        response = dialog.run()
        dialog.destroy()
        return response

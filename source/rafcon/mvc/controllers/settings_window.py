"""
.. module:: settings window
   :platform: Unix, Windows
   :synopsis: a module holding the controller for the configuration settings GUI

.. moduleauthor:: Benno Voggenreiter

"""
from gtk import ListStore
import gtk
import os

import yaml_configuration.config
from rafcon.utils import log
from rafcon.mvc.views.settings_window import SettingsWindowView
from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.config import global_gui_config
from rafcon.statemachine.config import global_config

from rafcon.mvc.gui_helper import react_to_event

logger = log.get_logger(__name__)


class SettingsWindowController(ExtendedController):
    """
    Controller handling the configuration settings GUI
    """

    def __init__(self, model, view):
        assert isinstance(view, SettingsWindowView)
        ExtendedController.__init__(self, model, view)
        # (setting. text, text_visible, toggle_activatable, togge_visible, text_editable, toggle_value)
        self.config_list_store = ListStore(str, str, bool, bool, bool, bool, bool)
        self.library_list_store = ListStore(str, str)
        self.gui_list_store = ListStore(str, str, bool, bool, bool, bool, bool)
        self.shortcut_list_store = ListStore(str, str)
        self._actual_entry = None
        self.lib_counter = 0
        self.gui_checkbox = gtk.CheckButton("GUI Config")
        self.core_checkbox = gtk.CheckButton("Core Config")

    def register_view(self, view):
        """Called when the View was registered"""
        self.view['add_btn'].connect('clicked', self.on_add_library)
        self.view["remove_btn"].connect('clicked', self.on_remove_library)
        self.view['value_toggle'].connect('toggled', self.on_checkbox_toggled, self.config_list_store)
        self.view['gui_value_toggle'].connect('toggled', self.on_checkbox_toggled, self.gui_list_store)

        self.view['value_text'].set_property('editable', True)
        self.view['value_text'].connect('edited', self.on_value_changed, self.config_list_store)
        self.view['value_text'].connect('editing-started', self.editing_started, self.view['value_text'],
                                        self.config_list_store, self.view['config_tree_view'])
        self.view['value_text'].connect('editing-canceled', self.editing_canceled)

        self.view['value_text1'].set_property('editable', True)
        self.view['value_text1'].connect('edited', self.on_value_changed, self.gui_list_store)
        self.view['value_text1'].connect('editing-started', self.editing_started, self.view['value_text1'],
                                         self.gui_list_store, self.view['gui_tree_view'])
        self.view['value_text1'].connect('editing-canceled', self.editing_canceled)

        self.view['value_text2'].set_property('editable', True)
        self.view['value_text2'].connect('edited', self.on_value_changed, self.shortcut_list_store)
        self.view['value_text2'].connect('editing-started', self.editing_started, self.view['value_text2'],
                                         self.shortcut_list_store, self.view["shortcut_tree_view"])
        self.view['value_text2'].connect('editing-canceled', self.editing_canceled)

        self.view['config_text3'].set_property('editable', True)
        self.view['config_text3'].connect('edited', self.on_name_changed)
        self.view['config_text3'].connect('editing-started', self.editing_started, self.view['config_text3'],
                                          self.library_list_store, self.view["library_tree_view"])
        self.view['config_text3'].connect('editing-canceled', self.editing_canceled)

        self.view['value_text3'].set_property('editable', True)
        self.view['value_text3'].connect('edited', self.on_value_changed, self.library_list_store)
        self.view['value_text3'].connect('editing-started', self.editing_started, self.view['value_text3'],
                                         self.library_list_store, self.view["library_tree_view"])
        self.view['value_text3'].connect('editing-canceled', self.editing_canceled)

        self.view['apply_button'].connect("clicked", self.on_save_and_apply_configurations)
        self.view['config_tree_view'].set_model(self.config_list_store)
        self.view['library_tree_view'].set_model(self.library_list_store)
        self.view['gui_tree_view'].set_model(self.gui_list_store)
        self.view['shortcut_tree_view'].set_model(self.shortcut_list_store)

        self.view['import_button'].connect("clicked", self.on_import_config)
        self.view['export_button'].connect('clicked', self.on_export_config)

        self.view['properties_window'].connect('delete_event', self.delete_event, self.view['properties_window'])

        self.view['cancel_button'].connect('clicked', self.on_delete)
        self.set_properties()
        self.set_label_paths()

    def on_add_library(self, *event):
        self.view['library_tree_view'].grab_focus()
        if react_to_event(self.view, self.view['library_tree_view'], event):
            lib_name = "<LIB_NAME_%s>" % self.lib_counter
            self.model.add_library(lib_name, "<LIB_PATH>")
            self.lib_counter += 1
            for row_num, iter_elem in enumerate(self.library_list_store):
                if iter_elem[0] == lib_name:
                    self.view['library_tree_view'].set_cursor(row_num)
            return True

    def on_remove_library(self, *event):
        self.view['library_tree_view'].grab_focus()
        if react_to_event(self.view, self.view['library_tree_view'], event):
            path = self.view["library_tree_view"].get_cursor()[0]
            if path is not None:
                key = self.library_list_store[int(path[0])][0]
                self.model.delete_library(key)
                if len(self.library_list_store) > 0:
                    self.view['library_tree_view'].set_cursor(min(path[0], len(self.library_list_store) - 1))
            return True

    def on_checkbox_toggled(self, renderer, path, actual_list_store):
        key = actual_list_store[int(path)][0]
        value = bool(actual_list_store[int(path)][6])
        value ^= True
        self.model.set_config_view_value(key, value)
        self.model.ignore_changes(key, value)

    def set_label_paths(self):
        """
        updating config path labels
        :return:
        """
        self.view['core_label'].set_text("Core Config Path: " + str(global_config.config_file_path))
        self.view['gui_label'].set_text("GUI Config Path: " + str(global_gui_config.config_file_path))

    def on_import_config(self, *args):
        """
        function to import config files.
        imported files will be automatically applied
        :param args:
        :return:
        """
        def handle_import(dialog_text, path_name):
            chooser = gtk.FileChooserDialog(dialog_text, None,
                                            gtk.FILE_CHOOSER_ACTION_SAVE,
                                            (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                             gtk.STOCK_OPEN, gtk.RESPONSE_ACCEPT))
            chooser.set_current_folder(path_name)
            response = chooser.run()
            if response == gtk.RESPONSE_ACCEPT:
                # watch out: chooser.get_filename() returns the whole file path inclusively the file name
                file_path, file_name = os.path.split(chooser.get_filename())
                check_dict = yaml_configuration.config.load_dict_from_yaml(chooser.get_filename())
                if check_dict["TYPE"] == "SM_CONFIG":
                    global_config.config_file_path = chooser.get_filename()
                    global_config.path = file_path
                    global_config.load(file_name, file_path)
                    logger.info("Imported Core Config from {0}" .format(file_name))
                elif check_dict["TYPE"] == "GUI_CONFIG":
                    global_gui_config.config_file_path = chooser.get_filename()
                    global_gui_config.path = file_path
                    global_gui_config.load(file_name, file_path)
                    logger.info("Imported GUI Config from {0}" .format(file_name))
                else:
                    logger.error("{0} is not a valid Config file" .format(file_name))
            elif response == gtk.RESPONSE_CANCEL:
                logger.warning("Import Config canceled!")
            chooser.destroy()

        pathname = global_config.path
        handle_import("Import Config Config from", pathname)

        self.model.detect_changes()  # to avoid refreshing everything even it is not necessary
        self.set_properties()       # updating the list views
        self.on_save_and_apply_configurations()  # applying setting -> refreshing necessary widgets
        self.set_label_paths()              # setting path labels

    def on_export_config(self, *args):
        """
        function to export config files
        :param args:
        :return:
        """
        response = self.config_chooser_dialog("Choose Config to export", "\nPlease select the Configs to export:\n")
        if response == gtk.RESPONSE_REJECT:
            return

        def handle_export(dialog_text, path_name, config_file):
            chooser = gtk.FileChooserDialog(dialog_text, None,
                                            gtk.FILE_CHOOSER_ACTION_SAVE,
                                            (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                             gtk.STOCK_SAVE_AS, gtk.RESPONSE_ACCEPT))
            chooser.set_current_folder(path_name)
            response = chooser.run()
            if response == gtk.RESPONSE_ACCEPT:
                name = chooser.get_filename()
                if not name:
                    logger.error("Config could not be exported! Invalid Config name!")
                else:
                    if ".yaml" not in name:
                        name += ".yaml"
                    new_config_file = open(name, mode='w+')
                    if config_file == global_config:
                        global_config.config_file_path = name
                        global_config.save_configuration()
                    else:
                        global_gui_config.config_file_path = name
                        global_gui_config.save_configuration()
                    new_config_file.close()
                    logger.info("Exported Config to {0}" .format(name))
            elif response == gtk.RESPONSE_CANCEL:
                logger.warning("Export Config canceled!")
            chooser.destroy()

        if self.core_checkbox.get_active():
            pathname = global_config.path
            handle_export("Saving Core Config as...", pathname, global_config)

        if self.gui_checkbox.get_active():
            pathname = global_gui_config.path
            handle_export("Saving GUI Config as...", pathname, global_gui_config)

    def config_chooser_dialog(self, title_text, label_text):
        """
        Dialog so select which config shall be exported
        :param title_text:
        :param label_text:
        :return:
        """
        dialog = gtk.Dialog(title_text, self.view["settings_window"],
                            flags=0, buttons=
                            (gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
                             gtk.STOCK_OK, gtk.RESPONSE_ACCEPT))
        label = gtk.Label(label_text)
        label.set_padding(xpad=10, ypad=5)
        dialog.vbox.pack_start(label)
        label.show()
        self.gui_checkbox = gtk.CheckButton("GUI Config")
        dialog.vbox.pack_start(self.gui_checkbox)
        self.gui_checkbox.show()
        self.core_checkbox = gtk.CheckButton("Core Config")
        self.core_checkbox.show()
        dialog.vbox.pack_start(self.core_checkbox)
        response = dialog.run()
        dialog.destroy()
        return response

    def set_properties(self):
        """
        A function to get the actual settings stored in config.yaml and gui_config.yaml
        :return:
        """
        self.model.get_settings()

    def delete_event(self, widget, event, data):
        """
        Called when the window gets destroyed, hides the window and loads the stored settings
        :param widget:
        :param event:
        :param data:
        :return:
        """
        data.hide()
        global_config.load('config.yaml', global_config.path)
        global_gui_config.load('gui_config.yaml', global_gui_config.path)
        self.set_properties()
        return gtk.TRUE

    def on_delete(self, widget):
        """
        Called when Cancel button is clicked, hides the window and loads the stored settings
        :param widget:
        :param event:
        :param data:
        :return:
        """
        self.view["properties_window"].hide()
        self.set_properties()

    @ExtendedController.observe('config_list', after=True)
    def update_config_settings(self, model, prop_name, info):
        """
        Updates the config_tree_view, triggered when config_list is changed
        :param model:
        :param prop_name:
        :param info:
        :return:
        """
        self.config_list_store.clear()
        for key in self.model.config_list:
            setting, value = key
            # (setting, text, text_visible, toggle_activatable, togge_visible, text_editable, toggle_state)
            if type(value) == bool:
                self.config_list_store.append((setting, value, False, True, True, False, value))
            else:
                self.config_list_store.append((setting, value, True, False, False, True, value))

    @ExtendedController.observe('config_library_list', after=True)
    def update_library_settings(self, model, prop_name, info):
        """
        Updates the library_tree_view, triggered when config_library_list is changed
        :param model:
        :param prop_name:
        :param info:
        :return:
        """
        self.library_list_store.clear()
        for key in self.model.config_library_list:
            setting, value = key
            self.library_list_store.append((setting, value))

    @ExtendedController.observe('config_gui_list', after=True)
    def update_gui_settings(self, model, prop_name, info):
        """
        Updates the gui_tree_view, triggered when config_gui_list is changed
        :param model:
        :param prop_name:
        :param info:
        :return:
        """
        self.gui_list_store.clear()
        for key in self.model.config_gui_list:
            setting, value = key
            # (setting, text, text_visible, toggle_activatable, togge_visible, text_editable, toggle_state)
            if type(value) == bool:
                self.gui_list_store.append((setting, value, False, True, True, False, value))
            else:
                self.gui_list_store.append((setting, value, True, False, False, True, value))

    @ExtendedController.observe('config_shortcut_list', after=True)
    def update_shortcut_settings(self, model, prop_name, info):
        """
        Updates the shortcut_tree_view, triggered when config_shortcut_list is changed
        :param model:
        :param prop_name:
        :param info:
        :return:
        """
        self.shortcut_list_store.clear()
        for key in self.model.config_shortcut_list:
            setting, value = key
            self.shortcut_list_store.append((setting, value))

    def editing_started(self, renderer, editable, path, actual_renderer, actual_list_store, actual_tree_view):
        """
        Callback method to connect entry-widget focus-out-event to the respective change-method.
        """
        if self.view['config_text3'] is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_name))
        elif actual_renderer is renderer:
            self._actual_entry = (editable, editable.connect('focus-out-event', self.change_value, actual_list_store, actual_tree_view))
        else:
            logger.error("Not registered Renderer was used")

    def editing_canceled(self, event):
        """ Callback method to disconnect entry-widget focus-out-event to the respective change-method.
        """
        if self._actual_entry is not None:
            self._actual_entry[0].disconnect(self._actual_entry[1])
            self._actual_entry = None

    def change_value(self, entry, event, actual_list_store, actual_tree_view):
        """ Change-value-method to set the value of actual selected (row) global variable.
        """
        logger.info("change value {0}".format(event.type))
        if actual_tree_view.get_cursor()[0] is None or \
                not actual_tree_view.get_cursor()[0][0]:
            return
        self.on_value_changed(entry, actual_tree_view.get_cursor()[0][0], text=entry.get_text(),
                              actual_list_store=actual_list_store)

    def change_name(self, entry, event):
        if self.view['library_tree_view'].get_cursor()[0] is None or \
                not self.view['library_tree_view'].get_cursor()[0][0]:
            return
        self.on_name_changed(entry, self.view['library_tree_view'].get_cursor()[0][0], text=entry.get_text())

    def on_name_changed(self, widget, path, text):
        """Triggered when a global variable's name is edited

        Updates the global variable's name only if different and already in list store.

        :param path: The path identifying the edited variable
        :param text: New variable name
        """
        # logger.info("changing name")
        old_key = self.library_list_store[int(path)][0]
        if old_key == text:
            return
        old_value = self.library_list_store[int(path)][1]
        try:
            self.model.set_library_key(text, old_key, old_value)
        except RuntimeError as e:
            logger.exception(e)

    def on_value_changed(self, widget, path, text, actual_list_store):
        """Triggered when a config value is edited.
        :param path: The path identifying the edited variable
        :param text: New variable value
        """
        if actual_list_store[int(path)][1] == text:
            return
        key = actual_list_store[int(path)][0]
        if actual_list_store == self.config_list_store or actual_list_store == self.library_list_store:
            data_type = type(global_config.get_config_value(key))
        elif actual_list_store == self.library_list_store:
            pass
        else:
            data_type = type(global_gui_config.get_config_value(key))
        try:
            if data_type == bool:
                if text == "False" or text == "false":
                    value = False
                elif text == "True" or text == "true":
                    value = True
                else:
                    logger.info("Invalid input: {0}".format(str(text)))
                    return
            elif data_type == int or data_type == float:
                value = data_type(text)
            else:
                value = text
            self.model.set_config_view_value(key, value)
        except RuntimeError as e:
            logger.exception(e)

    def on_save_and_apply_configurations(self, *args):
        """
        Triggered when Apply Button is clicked
        - stores all settings into the config files
        - pops up a te info message
        - updates library paths an shortcuts
        :param args:
        :return:
        """
        self.model.save_and_apply_config()
        self.popup_message()

    def popup_message(self):
        changes_str = ''
        if self.model.change_by_restart:
            message = gtk.MessageDialog(parent=self.view["properties_window"], flags=gtk.DIALOG_MODAL,
                                        type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_OK)
            message_string = "You must restart RAFCON to apply following changes: \n"
            if self.model.change_by_restart:
                for key in self.model.change_by_restart:
                    changes_str = ("%s\n%s" % (changes_str, key))
            message.set_markup("%s %s" % (message_string, changes_str))
            message.run()
            message.destroy()



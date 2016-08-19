"""
.. module:: settings window
   :platform: Unix, Windows
   :synopsis: a module holding the controller for the configuration settings GUI

.. moduleauthor:: Benno Voggenreiter

"""
from gtk import ListStore
import gtk

from rafcon.utils import log
from rafcon.mvc.views.settings_window import SettingsWindowView
from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.config import global_gui_config
from rafcon.statemachine.config import global_config
from rafcon.statemachine.library_manager import LibraryManager

logger = log.get_logger(__name__)


class SettingsWindowController(ExtendedController):
    """
    Controller handling the configuration settings GUI
    """

    def __init__(self, model, view, shortcut_manager):
        assert isinstance(view, SettingsWindowView)
        ExtendedController.__init__(self, model, view)
        self.config_list_store = ListStore(str, str)
        self.library_list_store = ListStore(str, str)
        self.gui_list_store = ListStore(str, str)
        self.shortcut_list_store = ListStore(str, str)
        self._actual_entry = None
        self.library_manager = LibraryManager()
        self.shortcut_manager = shortcut_manager

    def register_view(self, view):
        """Called when the View was registered"""
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

        self.view['properties_window'].connect('delete_event', self.delete_event, self.view['properties_window'])

        self.view['cancel_button'].connect('clicked', self.on_delete)
        self.set_properties()

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
        global_config.load('config.yaml', global_config.path)
        global_gui_config.load('gui_config.yaml', global_gui_config.path)
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
            self.config_list_store.append((setting, value))

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
            self.gui_list_store.append((setting, value))

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
        if actual_renderer is renderer:
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

    def on_value_changed(self, widget, path, text, actual_list_store):
        """Triggered when a config value is edited.
        :param path: The path identifying the edited variable
        :param text: New variable value
        """
        if actual_list_store[int(path)][1] == text:
            return
        key = actual_list_store[int(path)][0]
        if actual_list_store == self.config_list_store:
            list_nr = 0
        elif actual_list_store == self.gui_list_store:
            list_nr = 1
        elif actual_list_store == self.shortcut_list_store:
            list_nr = 2
        else:
            list_nr = 3
        if list_nr == 0 or list_nr == 3:
            data_type = type(global_config.get_config_value(key))
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
            self.model.set_config_view_value(key, value, list_nr)
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
        global_config.save_configuration()
        global_gui_config.save_configuration()
        message = gtk.MessageDialog(parent=self.view["properties_window"], flags=gtk.DIALOG_MODAL,
                                    type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_OK)
        message.set_markup("Please be aware that various changes in Core and GUI Config settings require"
                           " a restart! \n")
        message.run()
        message.destroy()
        self.library_manager.refresh_libraries()
        self.shortcut_manager.remove_shortcuts()
        self.shortcut_manager.update_shortcuts()


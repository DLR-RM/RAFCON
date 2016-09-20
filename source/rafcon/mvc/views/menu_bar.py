import gtk

from gtkmvc import View

from rafcon.mvc.utils import constants
from rafcon.mvc.gui_helper import set_label_markup
from rafcon.mvc.config import global_gui_config


class MenuBarView(View):
    builder = constants.get_glade_path("menu_bar.glade")
    top = 'menubar'

    buttons = {
        # -----------------------------------------------
        # File
        # -----------------------------------------------
        'new':                  constants.BUTTON_NEW,
        'open':                 constants.BUTTON_OPEN,
        'save':                 constants.BUTTON_SAVE,
        'save_as':              constants.BUTTON_SAVE,
        'save_state_as':        constants.BUTTON_SAVE,
        'menu_properties':      constants.BUTTON_PROP,
        'refresh_all':          constants.BUTTON_REFR,
        'refresh_libraries':    constants.BUTTON_REFR,
        'quit':                 constants.BUTTON_QUIT,
        # -----------------------------------------------
        # Edit
        # -----------------------------------------------
        'cut':                  constants.BUTTON_CUT,
        'copy':                 constants.BUTTON_COPY,
        'paste':                constants.BUTTON_PASTE,
        'is_start_state':       constants.BUTTON_SQUARE,
        'add':                  constants.BUTTON_ADD,
        'group':                constants.BUTTON_GROUP,
        'ungroup':              constants.BUTTON_UNGR,
        'substitute_state':     constants.BUTTON_REFR,
        'delete':               constants.BUTTON_DEL,
        'undo':                 constants.BUTTON_UNDO,
        'redo':                 constants.BUTTON_REDO,
        # -----------------------------------------------
        # View
        # -----------------------------------------------
        'data_flow_mode':       None,
        'show_data_flows':      None,
        'show_data_values':     None,
        'show_aborted_preempted': None,
        'expert_view':          constants.BUTTON_VIEW,
        'fullscreen':           None,
        # -----------------------------------------------
        # Execution
        # -----------------------------------------------
        'start':                constants.BUTTON_START,
        'start_from_selected':  constants.BUTTON_START_FROM_SELECTED_STATE,
        'pause':                constants.BUTTON_PAUSE,
        'stop':                 constants.BUTTON_STOP,
        'step_mode':            constants.BUTTON_STEPM,
        'step_into':            constants.BUTTON_STEP_INTO,
        'step_over':            constants.BUTTON_STEP_OVER,
        'step_out':             constants.BUTTON_STEP_OUT,
        'backward_step':        constants.BUTTON_BACKW,
        'run_to_selected':      constants.BUTTON_RUN_TO_SELECTED_STATE,
        # -----------------------------------------------
        # Help
        # -----------------------------------------------
        'about':                constants.BUTTON_ABOUT
        }

    def __init__(self, top_window):
        View.__init__(self)

        self.win = top_window['main_window']
        top_window.get_top_widget().add_accel_group(self['accelgroup1'])

        self.insert_accelerators = {}

        for menu_item_name in self.buttons.iterkeys():
            # set icon
            self.set_menu_item_icon(menu_item_name, self.buttons[menu_item_name])
            # set accelerator if in shortcuts dictionary with menu_item_name == key
            if menu_item_name in global_gui_config.get_config_value('SHORTCUTS'):
                shortcuts = global_gui_config.get_config_value('SHORTCUTS')[menu_item_name]
                if shortcuts:
                    self.set_menu_item_accelerator(menu_item_name, shortcuts[0])

    def set_menu_item_icon(self, menu_item_name, uni_code=None):
        menu_item = self[menu_item_name]

        if uni_code is not None:
            label = gtk.Label()
            set_label_markup(label, '&#x' + uni_code + ';',
                             font=constants.ICON_FONT, font_size=constants.FONT_SIZE_BIG)
            menu_item.set_image(label)
            menu_item.set_always_show_image(True)

    def set_menu_item_sensitive(self, menu_item_name, sensitive):
        self[menu_item_name].set_sensitive(sensitive)

    def set_menu_item_accelerator(self, menu_item_name, accel_code):
        menu_item = self[menu_item_name]
        if menu_item_name in self.insert_accelerators:
            key, mod = self.insert_accelerators[menu_item_name]
            menu_item.remove_accelerator(self['accelgroup1'], key, mod)
        key, mod = gtk.accelerator_parse(accel_code)
        menu_item.add_accelerator("activate", self['accelgroup1'], key, mod, gtk.ACCEL_VISIBLE)
        self.insert_accelerators[menu_item_name] = (key, mod)
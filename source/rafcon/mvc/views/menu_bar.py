import gtk

from gtkmvc import View

from rafcon.mvc.utils import constants
from rafcon.mvc.gui_helper import set_label_markup


class MenuBarView(View):
    builder = './glade/menu_bar.glade'
    top = 'menubar'

    buttons = {
        # -----------------------------------------------
        # File
        # -----------------------------------------------
        'new':                  constants.BUTTON_NEW,
        'open':                 constants.BUTTON_OPEN,
        'save':                 constants.BUTTON_SAVE,
        'save_as':              constants.BUTTON_SAVE,
        'menu_properties':      constants.BUTTON_PROP,
        'refresh_all':          constants.BUTTON_REFR,
        'refresh_libraries':    constants.BUTTON_REFR,
        'quit':                 constants.BUTTON_QUIT,
        # -----------------------------------------------
        # Edit
        # -----------------------------------------------
        'cut_selection':        constants.BUTTON_CUT,
        'copy_selection':       constants.BUTTON_COPY,
        'paste_clipboard':      constants.BUTTON_PASTE,
        'add_state':            constants.BUTTON_ADD,
        'group_states':         constants.BUTTON_GROUP,
        'ungroup_states':       constants.BUTTON_UNGR,
        'delete':               constants.BUTTON_DEL,
        'undo':                 constants.BUTTON_UNDO,
        'redo':                 constants.BUTTON_REDO,
        # -----------------------------------------------
        # View
        # -----------------------------------------------
        'expert_view':          constants.BUTTON_VIEW,
        # -----------------------------------------------
        # Execution
        # -----------------------------------------------
        'start':                constants.BUTTON_START,
        'start_from_selected_state': constants.BUTTON_START_FROM_SELECTED_STATE,
        'pause':                constants.BUTTON_PAUSE,
        'stop':                 constants.BUTTON_STOP,
        'step_mode':            constants.BUTTON_STEPM,
        'step_into':            constants.BUTTON_STEP_INTO,
        'step_over':            constants.BUTTON_STEP_OVER,
        'step_out':             constants.BUTTON_STEP_OUT,
        'backward_step':        constants.BUTTON_BACKW,
        'run_to_selected_state': constants.BUTTON_RUN_TO_SELECTED_STATE,
        # -----------------------------------------------
        # Help
        # -----------------------------------------------
        'about':                constants.BUTTON_ABOUT
        }

    def __init__(self, top_window):
        View.__init__(self)

        self.win = top_window['main_window']

        for key in self.buttons.iterkeys():
            self.set_image_for_menu_item(key)

    def set_image_for_menu_item(self, menu_item_name):
        menu_item = self[menu_item_name]

        label = gtk.Label()
        set_label_markup(label, '&#x' + self.buttons[menu_item_name] + ';',
                         font=constants.ICON_FONT, font_size=constants.FONT_SIZE_BIG)
        menu_item.set_image(label)
        menu_item.set_always_show_image(True)

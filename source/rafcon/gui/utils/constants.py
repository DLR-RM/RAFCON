# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import os
from collections import OrderedDict

from gi.repository import Gdk

from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE


def get_glade_path(glade_file):
    from os import path
    mvc_dir = path.dirname(path.dirname(__file__))
    return path.join(mvc_dir, "glade", glade_file)


INTERFACE_FONT = "Source Sans Pro"
ICON_FONT_FONTAWESOME = "FontAwesome5Free"
ICON_FONT_RAFCON = "RAFCON"
FONTS = [INTERFACE_FONT, ICON_FONT_FONTAWESOME, ICON_FONT_RAFCON]

FONT_SIZE_SMALL = "9"
FONT_SIZE_NORMAL = "10"
FONT_SIZE_BIG = "13"

LETTER_SPACING_NONE = "0"
LETTER_SPACING_05PT = "512"
LETTER_SPACING_075PT = "756"
LETTER_SPACING_1PT = "1024"
LETTER_SPACING_2PT = "2048"
LETTER_SPACING_3PT = "3072"

MOVE_CURSOR = Gdk.CursorType.FLEUR
SELECT_CURSOR = Gdk.CursorType.HAND1
CREATION_CURSOR = Gdk.CursorType.CROSS

KEYVALUE_CTRL_L = 65507
KEYVALUE_CTRL_R = 65508
KEYVALUE_ALT = 65513
KEYVALUE_ALT_GR = 65027
KEYVALUE_SHIFT_L = 65505
KEYVALUE_SHIFT_R = 65506
KEYVALUE_CONTEXT = 65383
KEYVALUE_TAB = 65289
MOVE_PORT_MODIFIER = Gdk.ModifierType.CONTROL_MASK
EXTEND_SELECTION_KEY = KEYVALUE_CTRL_L
EXTEND_SELECTION_KEY_ALT = KEYVALUE_CTRL_R
EXTEND_SELECTION_MODIFIER = Gdk.ModifierType.CONTROL_MASK
RUBBERBAND_MODIFIER = Gdk.ModifierType.SHIFT_MASK
RECURSIVE_RESIZE_MODIFIER = Gdk.ModifierType.CONTROL_MASK


MAX_VALUE_LABEL_TEXT_LENGTH = 7

MINIMUM_STATE_SIZE_FOR_DISPLAY = 5
MINIMUM_NAME_SIZE_FOR_DISPLAY = 10
MINIMUM_PORT_SIZE_FOR_DISPLAY = 4
MINIMUM_PORT_NAME_SIZE_FOR_DISPLAY = 4

GRID_SIZE = 10
PADDING_LEFT = 15
BORDER_WIDTH_STATE_SIZE_FACTOR = 25.
BORDER_WIDTH_OUTLINE_WIDTH_FACTOR = 35.
BORDER_WIDTH_LINE_WIDTH_FACTOR = 8.
BORDER_WIDTH_ROOT_STATE = 5.
BORDER_WIDTH_HIERARCHY_SCALE_FACTOR = 2.
MAIN_WINDOW_BORDER_WIDTH = 3
BORDER_WIDTH = 5
TAB_BORDER_WIDTH = 5
BORDER_WIDTH_TEXTVIEW = 10
BUTTON_BORDER_WIDTH = 5
BUTTON_MIN_WIDTH = 90
BUTTON_MIN_HEIGHT = 40
PADDING = 5
PANE_MARGIN = 46
ICON_SIZE_IN_PIXEL = 20
ICON_MARGIN = 10
MAXIMUM_CHILD_TO_PARENT_STATE_SIZE_RATIO = 50.
MAXIMUM_NAME_TO_PARENT_STATE_SIZE_RATIO = 8.

WINDOW_SIZE = {'MAIN_WINDOW': (1800, 900), 'LEFT_BAR_WINDOW': (300, 800), 'RIGHT_BAR_WINDOW': (300, 800),
               'CONSOLE_WINDOW': (800, 300)}

PANE_ID = OrderedDict([
    ('LEFT_BAR_DOCKED_POS', 'top_level_h_pane'),
    ('CONSOLE_DOCKED_POS', 'central_v_pane'),
    ('LEFT_BAR_INNER_PANE_POS', 'left_bar_paned'),
    ('RIGHT_BAR_DOCKED_POS', 'right_h_pane')
])

# a mapping of runtime config values for the hidden status of all three main bars (left, right and console)
# to hide functions (located in the main_window controller)
UNDOCKABLE_WINDOW_KEYS = ["LEFT_BAR", "RIGHT_BAR", "CONSOLE"]

DEFAULT_PANE_POS = {'LEFT_BAR_DOCKED_POS': 300,
                    'RIGHT_BAR_DOCKED_POS': 1000,
                    'CONSOLE_DOCKED_POS': 600,
                    'LEFT_BAR_INNER_PANE_POS': 400}

# The codes written down here are the codes provided on the font_awesome website
BUTTON_CHECK = "&#xf14a;"
BUTTON_COLLAPSE = "&#xf78c;"
BUTTON_SQUARE = "&#xf0c8;"
BUTTON_EXP = "&#xf065;"
BUTTON_EXCHANGE = "&#xf362;"
BUTTON_NEW = "&#xf15b;"
BUTTON_OPEN = "&#xf07c;"
BUTTON_SAVE = "&#xf0c7;"
BUTTON_PROP = "&#xf0ad;"
BUTTON_REFR = "&#xf021;"
BUTTON_BAKE = "&#xf187;"
BUTTON_CLOSE = "&#xf00d;"
BUTTON_QUIT = "&#xf057;"
BUTTON_CUT = "&#xf0c4;"
BUTTON_COPY = "&#xf0c5;"
BUTTON_PASTE = "&#xf0ea;"
BUTTON_ADD = "&#xf067;"
BUTTON_GROUP = "&#xf247;"
BUTTON_UNGR = "&#xf248;"
BUTTON_DEL = "&#xf1f8;"
BUTTON_UNDO = "&#xf0e2;"
BUTTON_REDO = "&#xf01e;"
BUTTON_VIEW = "&#xf06e;"
BUTTON_START = "&#xf04b;"
BUTTON_START_FROM_SELECTED_STATE = "&#xf2f5;"
BUTTON_RUN_TO_SELECTED_STATE = "&#xf2f6;"
BUTTON_PAUSE = "&#xf04c;"
BUTTON_STOP = "&#xf04d;"
BUTTON_STEPM = "&#xf54b;"
BUTTON_STEP_INTO = "&#xf019;"
BUTTON_STEP_OVER = "&#xf064;"
BUTTON_STEP_OUT = "&#xf093;"
BUTTON_BACKW = "&#xf3e5;"
BUTTON_ABOUT = "&#xf0a3;"
BUTTON_LEFTA = "&#xf0d9;"
BUTTON_RIGHTA = "&#xf0da;"
BUTTON_UPA = "&#xf0d8;"
BUTTON_DOWNA = "&#xf0d7;"
BUTTON_UNDOCK = "&#xf24d;"
SIGN_LIB = "&#xf02d;"
SIGN_ARROW = "&#xf0b2;"
ICON_SOURCE = "&#xf121;"
ICON_DLINK = "&#xf0c1;"
ICON_LLINK = "&#xf1e0;"
ICON_OVERV = "&#xf160;"
ICON_DESC = "&#xf036;"
ICON_SEMANTICS = "&#xf19c;"
ICON_TREE = "&#xf0e8;"
ICON_GLOB = "&#xf0ac;"
ICON_HIST = "&#xf254;"
ICON_EHIST = "&#xf1b3;"
ICON_NET = "&#xf0ec;"
ICON_STICKY = "&#xf08d;"
ICON_ERASE = "&#xf12d;"

ICON_STATE_EXECUTION = "&#xe904;"
ICON_STATE_HIERARCHY = "&#xe905;"
ICON_STATE_CONC_PREEMPTIVE = "&#xe906;"
ICON_STATE_CONS_BARRIER = "&#xe907;"

ICON_TREE_EXPAND = "&#xe900;"
ICON_TREE_FOLD = "&#xe901;"

ICON_REFRESH_LIBS = "&#xe902;"
ICON_REFRESH_SELECTED = "&#xe903;"

ICONS_WITH_BOLD_FACE = [BUTTON_NEW, BUTTON_OPEN, BUTTON_SAVE, BUTTON_QUIT, BUTTON_COPY, BUTTON_STEPM]
ICONS_IN_RAFCON_FONT = [ICON_STATE_EXECUTION, ICON_STATE_HIERARCHY, ICON_STATE_CONC_PREEMPTIVE, ICON_STATE_CONS_BARRIER,
                        ICON_TREE_EXPAND, ICON_TREE_FOLD, ICON_REFRESH_LIBS, ICON_REFRESH_SELECTED]

ICON_STATE_FILL_FACTOR = 0.8

RAFCON_INSTANCE_LOCK_FILE_PATH = os.path.join(RAFCON_TEMP_PATH_BASE, 'lock')

complex_actions = ['change_root_state_type', 'change_state_type',
                   'substitute_state', 'group_states', 'ungroup_state',
                   'paste', 'cut', 'undo/redo']

execution_running_style_class = "execution-running"

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
from gi.repository import Gtk
from gi.repository import Gdk
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE


def get_glade_path(glade_file):
    from os import path
    mvc_dir = path.dirname(path.dirname(__file__))
    return path.join(mvc_dir, "glade", glade_file)


FONTS = ["DIN Next LT Pro", "FontAwesome"]
INTERFACE_FONT = FONTS[0]
ICON_FONT = FONTS[1]

FONT_SIZE_SMALL = "10"
FONT_SIZE_NORMAL = "11"
FONT_SIZE_BIG = "14"

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

PANE_ID = {'LEFT_BAR_DOCKED_POS': 'top_level_h_pane',
           'RIGHT_BAR_DOCKED_POS': 'right_h_pane',
           'CONSOLE_DOCKED_POS': 'central_v_pane',
           'LEFT_BAR_INNER_PANE_POS': 'left_bar_paned'}

# a mapping of runtime config values for the hidden status of all three main bars (left, right and console)
# to hide functions (located in the main_window controller)
UNDOCKABLE_WINDOW_KEYS = ["LEFT_BAR", "RIGHT_BAR", "CONSOLE"]

DEFAULT_PANE_POS = {'LEFT_BAR_DOCKED_POS': 300,
                    'RIGHT_BAR_DOCKED_POS': 1000,
                    'CONSOLE_DOCKED_POS': 600,
                    'LEFT_BAR_INNER_PANE_POS': 400}

# The codes written down here are the codes provided on the font_awesome website
BUTTON_CHECK = "f046"
BUTTON_COLLAPSE = "f066"
BUTTON_SQUARE = "f096"
BUTTON_EXP = "f065"
BUTTON_EXCHANGE = "f0ec"
BUTTON_NEW = "f016"
BUTTON_OPEN = "f115"
BUTTON_SAVE = "f0c7"
BUTTON_PROP = "f0ad"
BUTTON_REFR = "f021"
BUTTON_BAKE = "f187"
BUTTON_CLOSE = "f00d"
BUTTON_QUIT = "f08b"
BUTTON_CUT = "f0c4"
BUTTON_COPY = "f0c5"
BUTTON_PASTE = "f0ea"
BUTTON_ADD = "f067"
BUTTON_GROUP = "f247"
BUTTON_UNGR = "f248"
BUTTON_DEL = "f1f8"
BUTTON_UNDO = "f0e2"
BUTTON_REDO = "f01e"
BUTTON_VIEW = "f06e"
BUTTON_START = "f04b"
BUTTON_START_FROM_SELECTED_STATE = "f08b"
BUTTON_RUN_TO_SELECTED_STATE = "f090"
BUTTON_PAUSE = "f04c"
BUTTON_STOP = "f04d"
BUTTON_STEPM = "f050"
BUTTON_STEP_INTO = "f149"
BUTTON_STEP_OVER = "f051"
BUTTON_STEP_OUT = "f148"
BUTTON_BACKW = "f048"
BUTTON_ABOUT = "f0a3"
BUTTON_LEFTA = "f0d9"
BUTTON_RIGHTA = "f0da"
BUTTON_UPA = "f0d8"
BUTTON_DOWNA = "f0d7"
BUTTON_UNDOCK = "f24d"
SIGN_LIB = "f02d"
SIGN_ARROW = "f047"
ICON_SOURCE = "f121"
ICON_DLINK = "f0c1"
ICON_LLINK = "f1e0"
ICON_OVERV = "f160"
ICON_DESC = "f036"
ICON_SEMANTICS = "f19c"
ICON_TREE = "f0e8"
ICON_GLOB = "f0ac"
ICON_HIST = "f254"
ICON_EHIST = "f1b3"
ICON_NET = "f0ec"
ICON_STICKY = "f08d"
ICON_ERASE = 'f12d'

RAFCON_INSTANCE_LOCK_FILE_PATH = os.path.join(RAFCON_TEMP_PATH_BASE, 'lock')

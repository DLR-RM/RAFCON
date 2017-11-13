# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
from rafcon.gui import glade
from gtkmvc import View

from rafcon.gui.utils import constants


class SemanticDataEditorView(View):

    """ The view of the Semantic Data Editor widget

    """

    builder = glade.get_glade_path("semantic_data_editor.glade")
    top = 'semantic_data_vbox'

    def __init__(self):
        super(SemanticDataEditorView, self).__init__()

        self['delete_entry'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['new_dict_entry'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['new_entry'].set_border_width(constants.BUTTON_BORDER_WIDTH)



# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc3.view import View

from rafcon.gui import glade
from rafcon.gui.utils import constants
from rafcon.gui.helpers import label


class GlobalVariableEditorView(View):
    builder = glade.get_glade_path("global_variable_editor_widget.glade")
    top = 'global_variable_vbox'

    def __init__(self):
        View.__init__(self)

        self['new_global_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['delete_global_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['lock_global_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['unlock_global_variable_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self.scrollbar_widget = self['scroller']

        label.ellipsize_labels_recursively(self['global_variables_toolbar'])

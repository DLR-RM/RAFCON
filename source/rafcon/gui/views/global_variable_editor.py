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

from rafcon.design_patterns.mvc.view import View

from rafcon.gui import glade
from rafcon.gui.utils import constants
from rafcon.gui.utils.gtk_utils import set_all_margins
from rafcon.gui.helpers import label


class GlobalVariableEditorView(View):
    def __init__(self):
        super().__init__(builder_filename=glade.get_glade_path('global_variable_editor_widget.ui'), parent='global_variable_vbox')
        set_all_margins(self['new_global_variable_button'], constants.BUTTON_BORDER_WIDTH)
        set_all_margins(self['delete_global_variable_button'], constants.BUTTON_BORDER_WIDTH)
        set_all_margins(self['lock_global_variable_button'], constants.BUTTON_BORDER_WIDTH)
        set_all_margins(self['unlock_global_variable_button'], constants.BUTTON_BORDER_WIDTH)
        self.scrollbar_widget = self['scroller']
        label.ellipsize_labels_recursively(self['global_variables_toolbar'])

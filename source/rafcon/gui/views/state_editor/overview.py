# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from rafcon.design_patterns.mvc.view import View

from rafcon.gui import glade
from rafcon.gui.utils import constants


class StateOverviewView(View):
    def __init__(self):
        super().__init__(builder_filename=glade.get_glade_path('state_overview_widget.glade'), parent='properties_widget')
        self['properties_widget'].set_border_width(constants.PADDING_LEFT)

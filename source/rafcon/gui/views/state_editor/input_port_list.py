# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc import View
from rafcon.gui.utils import constants


class InputPortsListView(View):
    builder = constants.get_glade_path("input_ports_list_widget.glade")
    top = 'input_ports_tree_view'

    def __init__(self):
        View.__init__(self)

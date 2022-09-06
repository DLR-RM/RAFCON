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

from rafcon.gui.views.utils.tree import TreeView

from rafcon.gui import glade


class OutputPortsListView(TreeView):
    def __init__(self):
        super().__init__(builder_filename=glade.get_glade_path('output_ports_list_widget.glade'), parent='output_ports_tree_view')

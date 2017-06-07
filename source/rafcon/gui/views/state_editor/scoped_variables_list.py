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


class ScopedVariablesListView(TreeView):
    builder = glade.get_glade_path("scoped_variables_list_widget.glade")
    top = 'scoped_variables_tree_view'

    def __init__(self):
        super(ScopedVariablesListView, self).__init__()

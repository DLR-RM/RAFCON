# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>

from gtkmvc3.view import View


class TreeView(View):
    builder = None
    top = None

    def __init__(self):
        super(TreeView, self).__init__()
        self.scrollbar_widget = None

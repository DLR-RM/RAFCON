# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gtkmvc3.view import View
from rafcon.gui import glade


class PreferencesWindowView(View):
    builder = glade.get_glade_path("preferences_window.glade")
    top = 'preferences_window'

    def __init__(self):
        View.__init__(self)


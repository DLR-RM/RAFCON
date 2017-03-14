# coding=utf-8

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

import gtk
import rafcon


class AboutDialogView(gtk.AboutDialog):
    def __init__(self):
        gtk.AboutDialog.__init__(self)

        self.set_program_name("RAFCON")
        self.set_version(rafcon.__version__)
        self.set_authors(("Rico Belder", "Sebastian Brunner", "Franz Steinmetz", "Michael Vilzmann", "Lukas Becker",
                          "Annika Wollschläger", "Benno Voggenreiter", "Matthias Büttner", "Mahmoud Akl"))
        # TODO: set copyright/license
        # self.set_copyright("Copyright: DLR")
        # self.set_license("Copyright: DLR")
        self.set_website("https://rmintra01.robotic.dlr.de/wiki/RAFCON")

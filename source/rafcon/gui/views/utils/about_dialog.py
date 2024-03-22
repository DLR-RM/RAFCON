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

from gi.repository import Gtk
from gi.repository import GdkPixbuf
import rafcon


class AboutDialogView(Gtk.AboutDialog):
    def __init__(self):
        super(AboutDialogView, self).__init__()

        self.set_program_name("RAFCON")
        self.set_version(rafcon.__version__)
        self.set_authors(("Rico Belder", "Sebastian Brunner", "Franz Steinmetz", "Michael Vilzmann", "Lukas Becker",
                          "Annika Wollschläger", "Benno Voggenreiter", "Matthias Büttner", "Mahmoud Akl"))
        self.set_copyright("DLR")
        self.set_license("Eclipse Public License 1.0")
        self.set_logo(GdkPixbuf.Pixbuf.new_from_file('documents/logo/bitmap/RAFCON_Logo_Farbe_RGB_negativ_small.png'))
        self.set_website("https://github.com/DLR-RM/RAFCON")

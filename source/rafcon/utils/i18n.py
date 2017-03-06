# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import locale
import gettext

_ = gettext.gettext


def setup_l10n():
    locale.setlocale(locale.LC_ALL, '')
    gettext.bindtextdomain('rafcon', 'locale')
    gettext.textdomain('rafcon')


def setup_l10n_gtk():
    import gtk
    for module in (gettext, gtk.glade):
        module.bindtextdomain('rafcon', 'locale')
        module.textdomain('rafcon')

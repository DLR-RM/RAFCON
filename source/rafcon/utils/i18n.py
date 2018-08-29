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
# Franz Steinmetz <franz.steinmetz@dlr.de>

import locale
import gettext

from rafcon.utils.resources import resource_filename
from rafcon.utils import log

logger = log.get_logger(__name__)


def setup_l10n():
    try:
        locale.setlocale(locale.LC_ALL, '')
    except locale.Error as e:
        logger.warning("Cannot setup translations: {}".format(e))

    localedir = resource_filename('rafcon', 'locale')

    # Install gettext globally: Allows to use _("my string") without further imports
    gettext.install('rafcon', localedir)

    # Required for glade and the GtkBuilder
    locale.bindtextdomain('rafcon', localedir)
    locale.textdomain('rafcon')

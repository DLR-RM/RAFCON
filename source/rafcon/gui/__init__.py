# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import rafcon.gui.interface

try:
    import gi
    gi.require_version('Gtk', '3.0')
    gi.require_version('PangoCairo', '1.0')
    gi.require_version('GtkSource', '3.0')
except (ImportError, ValueError) as e:
    from rafcon.utils import log
    logger = log.get_logger(__name__)
    logger.warning("Could not import all packages required for opening the GUI: {}".format(e))

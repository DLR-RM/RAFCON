# Copyright (C) 2014-2017 DLR
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

from pkg_resources import get_distribution, DistributionNotFound

try:
    __version__ = get_distribution("rafcon").version
except DistributionNotFound:
    # the version cannot be found via pip which means rafcon was not installed yet on the system via the setup.py or pip
    # thus try to parse it from the version.py file directly

    import os
    file_path = os.path.join(os.path.dirname(__file__), "../../VERSION")
    try:
        with open(file_path, "r") as f:
            content = f.read().splitlines()
            # append a1 to signal that this is not a release version but a alpha/develop version
            # a1 is used as e.g. dev1 is not supported by the StrictVersion class
            __version__ = content[0] + "a1"
    except Exception as e:
        from rafcon.utils import log
        logger = log.get_logger(__name__)
        logger.error(str(e))
        logger.error("Setting the rafcon version to 'unknown' ... ")
        # this case must not happen, else state machines cannot be loaded from the file system
        __version__ = "unknown"

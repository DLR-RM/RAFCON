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

__all__ = ["gui", "core", "utils"]

try:
    __version__ = get_distribution("rafcon").version
except DistributionNotFound:
    # the version cannot be found via pip which means rafcon was not installed yet on the system via the setup.py or pip
    # thus try to parse it from the version.py file directly
    import os
    file_path = os.path.join(os.path.dirname(__file__), "../../version.py")
    if os.path.exists(file_path):
        import imp
        foo = imp.load_source('version', file_path)
        __version__ = foo.version
    else:
        # this case must not happen, else state machines cannot be loaded from the file system
        __version__ = "unknown"

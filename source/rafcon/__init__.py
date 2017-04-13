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
    __version__ = "unknown"

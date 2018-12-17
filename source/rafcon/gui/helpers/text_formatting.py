# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from future.utils import string_types


def limit_string(text, max_length, separator="&#x2026;"):
    from xml.sax.saxutils import escape
    import math
    if isinstance(text, string_types) and len(text) > max_length:
        max_length = int(max_length)
        half_length = float(max_length - 1) / 2
        return escape(text[:int(math.ceil(half_length))]) + separator + escape(text[-int(math.floor(half_length)):])
    return escape(text)


def format_default_folder_name(folder_name):
    folder_name = folder_name.replace(' ', '_')
    return folder_name.lower()


def format_folder_name_human_readable(folder_name):
    return folder_name.replace('_', ' ')

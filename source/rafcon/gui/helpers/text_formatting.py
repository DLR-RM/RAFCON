# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>


def limit_string(text, max_length, seperator="&#x2026;"):
    from xml.sax.saxutils import escape
    if isinstance(text, basestring) and len(text) > max_length:
        half_length = (max_length - 1) / 2
        return escape(text[:half_length]) + seperator + escape(text[-half_length:])
    return escape(text)

# Copyright (C) 2026 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html

"""In-application drag & drop payload registry

GTK4 drag sources provide their content eagerly when the drag starts, while RAFCON's
GTK3 implementation created the dragged state lazily when the data was requested at
drop time (the drop target - a container state - is only selected during the drag).

As all RAFCON drags are in-application, the drag sources (library tree, state icons)
register a callable here which produces the state to be inserted. The graphical editor
drop target consumes it when the drop actually happens.
"""

_drag_payload_provider = None


def set_drag_payload_provider(provider):
    """Register a callable returning the core state to be inserted on drop"""
    global _drag_payload_provider
    _drag_payload_provider = provider


def take_drag_payload_provider():
    """Return and clear the registered payload provider"""
    global _drag_payload_provider
    provider, _drag_payload_provider = _drag_payload_provider, None
    return provider


def clear_drag_payload_provider():
    global _drag_payload_provider
    _drag_payload_provider = None

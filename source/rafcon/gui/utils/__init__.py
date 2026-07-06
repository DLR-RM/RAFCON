# Copyright (C) 2014-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>


def wait_for_gui():
    from gi.repository import GLib
    context = GLib.MainContext.default()
    # The number of iterations is bounded: with the GLib-backed asyncio event loop
    # (gi.events, required by gaphas 5), sources of pending asyncio tasks can keep the
    # context "pending" forever while the loop is not running (e.g. during shutdown).
    for _ in range(1000):
        if not context.pending():
            break
        context.iteration(False)

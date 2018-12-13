# Copyright (C) 2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Rico Belder <rico.belder@dlr.de>

""" The sub-package is used to collect modules and classes that provide functionality to backup, backup and version
RAFCON state machine or elements.
It is a question if the modification history and actions also should be moved here, too.

In general this sub-package on the one hand is used to encapsulate features like session backup and auto backup of
state machines better but also to integrate those in the bigger picture. e.g the auto_backup module implemented
as model is becoming a pure Observer and will be moved in the new module rafcon.gui.backup.state_machine with
class AutoBackupStateMachine.
"""

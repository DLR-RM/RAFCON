# Copyright (C) 2016-2017 DLR
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

from collections import namedtuple

MetaSignalMsg = namedtuple('MetaSignalMsg', ['origin', 'change', 'affects_children', 'notification'])
MetaSignalMsg.__new__.__defaults__ = (False, None)  # Make last two parameters optional

ActionSignalMsg = namedtuple('ActionSignalMsg', ['action', 'origin', 'action_parent_m', 'affected_models', 'after',
                                                 'kwargs', 'result'])
ActionSignalMsg.__new__.__defaults__ = ({}, None)

StateTypeChangeSignalMsg = namedtuple('StateTypeChangeSignalMsg', ['new_state_m'])

Notification = namedtuple('Notification', ['model', 'prop_name', 'info'])

SelectionChangedSignalMsg = namedtuple('SelectionChangedSignalMsg', ['method_name', 'new_selection', 'old_selection',
                                                                     'affected_core_element_classes'])
FocusSignalMsg = namedtuple('FocusSignalMsg', ['new_focus', 'old_focus'])

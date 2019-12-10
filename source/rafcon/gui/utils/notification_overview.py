# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>

from gtkmvc3.observer import NTInfo

from rafcon.utils import constants
from rafcon.gui.models.signals import MetaSignalMsg, ActionSignalMsg


class NotificationOverview(object):
    empty_info = {'before': True, 'model': None, 'method_name': '', 'instance': None,
                  'prop_name': None, 'args': (), 'kwargs': {}, 'info': {}}

    def __init__(self, info=None):
        if info is None:
            info = NTInfo('before', **self.empty_info)
        self.info = info
        self.origin = self.extract_origin(info)
        self._type = self.extract_type(info)

    def prepare_destruction(self):
        self.info = None
        self.origin = None

    @staticmethod
    def extract_type(info):
        possible_types = NTInfo._NTInfo__ONE_REQUESTED
        for type_ in possible_types:
            if type_ in info and info[type_]:
                return type_

    @staticmethod
    def extract_origin(info):
        info_origin = info

        # This handles before/after notifications
        if "method_name" in info_origin and info_origin["method_name"]:
            while info_origin["method_name"].endswith("_change"):  # e.g. root_state_change, state_change, outcome_change
                info_origin = info_origin.kwargs

        # This handles signal notifications
        elif "signal" in info_origin:
            message = info_origin["arg"]

            # Nested MetaSignalMsg
            if hasattr(message, "notification") and message.notification:
                info_origin = message.notification.info

        notification_type = NotificationOverview.extract_type(info_origin)
        info_origin = NTInfo(notification_type, **info_origin)
        return info_origin

    @property
    def type(self):
        return self._type

    def get_cause(self):
        if self.type in ["before", "after"]:
            return self.origin.method_name
        if self.type == "signal":
            if isinstance(self.origin.arg, ActionSignalMsg):
                return self.origin.arg.action
            if isinstance(self.origin.arg, MetaSignalMsg):
                return self.origin.arg.origin

    def get_method_args(self):
        if self.type == "signal":
            return []
        return self.origin.args

    def get_method_kwargs(self):
        if self.type == "signal":
            if isinstance(self.origin.arg, ActionSignalMsg):
                return self.origin.arg.kwargs
            if isinstance(self.origin.arg, MetaSignalMsg):
                return {}
        return self.origin.kwargs

    def get_affected_model(self):
        return self.origin.model

    def get_affected_core_element(self):
        if self.type == "signal":
            if isinstance(self.origin.arg, ActionSignalMsg):
                return self.origin.arg.action_parent_m.core_element
            if isinstance(self.origin.arg, MetaSignalMsg):
                return self.origin.model.core_element
        if self.type in ["before", "after"]:
            return self.origin.instance

    def get_affected_property(self):
        return self.origin.prop_name

    def get_result(self):
        assert self.type == "after", "Only after notifications carry a result"
        return self.origin.result

    def get_signal_message(self):
        assert self.type == "signal", "Cannot retrieve message from non-signal notification"
        return self.origin.arg

    def get_change(self):
        if self.type in ["before", "after"]:
            if self.info.method_name.endswith("_change"):
                return self.info.method_name

    def operation_started(self):
        return "before" == self.type

    def operation_finished(self):
        if "after" != self.type:
            return False
        return not isinstance(self.get_result(), Exception)

    def caused_modification(self):
        return self.get_cause() not in constants.BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS + ["marked_dirty"]

    def __str__(self):
        text = \
            "{type} notification:\n" \
            "* Cause: {cause}(args={args}, kwargs={kwargs})\n" \
            "* Model: {model} / Core: {core} / Property: {prop}".format(
                type=self.type,
                cause=self.get_cause(), args=self.get_method_args(), kwargs=self.get_method_kwargs(),
                model=self.get_affected_model(), core=self.get_affected_core_element(), prop=self.get_affected_property(),
            )
        if self.type == "after":
            text += "\n* Result: {result}".format(result=self.get_result())
        if self.get_change():
            text += "\n* Wrapped in: {change}".format(change=self.get_change())
        return text

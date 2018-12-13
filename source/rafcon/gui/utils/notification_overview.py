from __future__ import print_function
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

from builtins import str
import datetime
import time
from rafcon.utils import constants
from rafcon.gui.models.signals import MetaSignalMsg, ActionSignalMsg


EXECUTION_TRIGGERED_METHODS = constants.BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS


def is_execution_status_update_notification_from_state_machine_model(prop_name, info):
    # avoid updates or checks because of execution status updates -> prop_name == 'state_machine'
    if prop_name == 'state_machine' and 'kwargs' in info and 'prop_name' in info['kwargs'] and \
            info['kwargs']['prop_name'] in ['states', 'state']:
        if 'method_name' in info['kwargs'] and info['kwargs']['method_name'] in EXECUTION_TRIGGERED_METHODS or \
                'kwargs' in info['kwargs'] and 'method_name' in info['kwargs']['kwargs'] and \
                info['kwargs']['kwargs']['method_name'] in EXECUTION_TRIGGERED_METHODS:
            return True
    if prop_name == 'state_machine' and 'method_name' in info and info['method_name'] == '_add_new_execution_history':
        return True


def is_execution_status_update_notification_from_state_model(prop_name, info):
    # avoid updates or checks because of execution status updates -> prop_name in ['state', 'states']
    if prop_name == 'states' and 'kwargs' in info and 'method_name' in info['kwargs'] and \
            info['kwargs']['method_name'] in EXECUTION_TRIGGERED_METHODS or \
            prop_name == 'state' and 'method_name' in info and info['method_name'] in EXECUTION_TRIGGERED_METHODS:
        return True


def is_execution_status_update_notification(prop_name, info):
    return is_execution_status_update_notification_from_state_machine_model(prop_name, info) or \
           is_execution_status_update_notification_from_state_model(prop_name, info)


class NotificationOverview(dict):
    empty_info = {'before': True, 'model': None, 'method_name': None, 'instance': None,
                  'prop_name': None, 'args': (), 'kwargs': {}, 'info': {}}
    # TODO comment
    _count = 0
    _generation_time = 0.

    def __init__(self, info=None, with_prints=False, initiator_string=None):
        from weakref import ref
        NotificationOverview._count += 1
        start_time = time.time()
        if info is None:
            info = self.empty_info
        self.initiator = initiator_string
        self.info = info
        # self._info = ref(info)
        self.__type = 'before'
        if 'after' in info:
            self.__type = 'after'
        elif 'signal' in info:
            self.__type = 'after'
        self.with_prints = with_prints
        s, overview_dict = self.get_nice_info_dict_string(info)
        self.time_stamp = datetime.datetime.now()
        self._overview_dict = overview_dict
        dict.__init__(self, overview_dict)
        self.__description = s
        if self.with_prints:
            print("\nNotificationOverview {}\n".format(str(self)))
        NotificationOverview._generation_time += time.time() - start_time
    #     self.store_debug_log_file("\nNotificationOverview {}\n".format(str(self)))
    #
    # def store_debug_log_file(self, string):
    #     with open(constants.RAFCON_TEMP_PATH_BASE + '/NO_debug_log_file.txt', 'a+') as f:
    #         f.write(string)
    #     f.closed

    def prepare_destruction(self):
        self.info = None
        self. _overview_dict.clear()
        dict.clear(self)

    def __str__(self):
        if self.initiator is not None:
            return "Initiator: {}\n".format(self.initiator) + self.__description
        else:
            return self.__description

    def __setitem__(self, key, value):
        if key in ['info', 'model', 'prop_name', 'instance', 'method_name', 'level']:
            dict.__setitem__(self, key, value)

    @property
    def type(self):
        return self.__type

    def update(self, E=None, **F):
        if E is not None:
            for key in E.keys:
                if key not in ['info', 'model', 'prop_name', 'instance', 'method_name', 'level']:
                    E.pop(key)
            dict.update(self, E)

    def print_overview(self, overview=None):
        print(self)

    def get_nice_info_dict_string(self, info, level='\t', overview=None):
        """ Inserts all elements of a notification info-dictionary of gtkmvc3 or a Signal into one string and indicates
        levels of calls defined by 'kwargs'. Additionally, the elements get structured into a dict that holds all levels
        of the general notification key-value pairs in faster accessible lists. The dictionary has the element 'type'
        and the general elements {'model': [], 'prop_name': [], 'instance': [], 'method_name': [], 'args': [],
        'kwargs': []}) plus specific elements according the type. Type is always one of the following list
        ['before', 'after', 'signal'].
        """
        def get_nice_meta_signal_msg_tuple_string(meta_signal_msg_tuple, level, overview):
            meta_signal_dict = {}
            # origin
            s = "\n{0}origin={1}".format(level + "\t", meta_signal_msg_tuple.origin)
            meta_signal_dict['origin'] = meta_signal_msg_tuple.origin
            # change
            s += "\n{0}change={1}".format(level + "\t", meta_signal_msg_tuple.change)
            meta_signal_dict['change'] = meta_signal_msg_tuple.change
            # affects_children
            s += "\n{0}affects_children={1}".format(level + "\t", meta_signal_msg_tuple.affects_children)
            meta_signal_dict['affects_children'] = meta_signal_msg_tuple.affects_children
            overview['signal'].append(meta_signal_dict)

            # notification (tuple)
            notification_dict = {}
            meta_signal_dict['notification'] = notification_dict
            if meta_signal_msg_tuple.notification is None:
                s += "\n{0}notification={1}".format(level + "\t", meta_signal_msg_tuple.notification)
            else:
                s += "\n{0}notification=Notification(".format(level + "\t")
                # model
                notification_dict['model'] = meta_signal_msg_tuple.notification.model
                s += "\n{0}model={1}".format(level + "\t\t", meta_signal_msg_tuple.notification.model)
                # prop_name
                notification_dict['prop_name'] = meta_signal_msg_tuple.notification.prop_name
                s += "\n{0}prop_name={1}".format(level + "\t\t", meta_signal_msg_tuple.notification.prop_name)
                # info
                notification_dict['info'] = meta_signal_msg_tuple.notification.info
                overview['kwargs'].append(meta_signal_msg_tuple.notification.info)
                s += "\n{0}info=\n{1}{0}\n".format(level + "\t\t",
                                               self.get_nice_info_dict_string(meta_signal_msg_tuple.notification.info,
                                                                              level+'\t\t\t',
                                                                              overview))
            return s

        def get_nice_action_signal_msg_tuple_string(meta_signal_msg_tuple, level, overview):
            meta_signal_dict = {}
            # after
            s = "\n{0}after={1}".format(level + "\t", meta_signal_msg_tuple.after)
            meta_signal_dict['after'] = meta_signal_msg_tuple.after
            # action
            s += "\n{0}action={1}".format(level + "\t", meta_signal_msg_tuple.action)
            meta_signal_dict['action'] = meta_signal_msg_tuple.action
            # origin
            s += "\n{0}origin={1}".format(level + "\t", meta_signal_msg_tuple.origin)
            meta_signal_dict['origin'] = meta_signal_msg_tuple.origin
            # origin
            s += "\n{0}action_parent_m={1}".format(level + "\t", meta_signal_msg_tuple.action_parent_m)
            meta_signal_dict['action_parent_m'] = meta_signal_msg_tuple.origin
            # change
            s += "\n{0}affected_models={1}".format(level + "\t", meta_signal_msg_tuple.affected_models)
            meta_signal_dict['affected_models'] = meta_signal_msg_tuple.affected_models
            if meta_signal_msg_tuple.after:
                s += "\n{0}result={1}".format(level + "\t", meta_signal_msg_tuple.result)
                meta_signal_dict['result'] = meta_signal_msg_tuple.result

            return s


        overview_was_none = False
        if overview is None:
            overview_was_none = True
            overview = dict({'model': [], 'prop_name': [], 'instance': [], 'method_name': [], 'args': [], 'kwargs': []})
            overview['others'] = []
            overview['info'] = []
            if 'before' in info:
                overview['type'] = 'before'
            elif 'after' in info:
                overview['type'] = 'after'
                overview['result'] = []
            else:  # 'signal' in info:
                overview['type'] = 'signal'
                overview['signal'] = []

        if ('after' in info or 'before' in info or 'signal' in info) and 'model' in info:
            if 'before' in info:
                s = "{0}'before': {1}".format(level, info['before'])
            elif 'after' in info:
                s = "{0}'after': {1}".format(level, info['after'])
            else:
                s = "{0}'signal': {1}".format(level, info['signal'])
        else:
            return str(info)
        overview['info'].append(info)
        # model
        s += "\n{0}'model': {1}".format(level, info['model'])
        overview['model'].append(info['model'])
        # prop_name
        s += "\n{0}'prop_name': {1}".format(level, info['prop_name'])
        overview['prop_name'].append(info['prop_name'])
        if not overview['type'] == 'signal':
            # instance
            s += "\n{0}'instance': {1}".format(level, info['instance'])
            overview['instance'].append(info['instance'])
            # method_name
            s += "\n{0}'method_name': {1}".format(level, info['method_name'])
            overview['method_name'].append(info['method_name'])
            # args
            s += "\n{0}'args': {1}".format(level, info['args'])
            overview['args'].append(info['args'])

            overview['kwargs'].append(info['kwargs'])
            if overview['type'] == 'after':
                overview['result'].append(info['result'])
            # kwargs
            s += "\n{0}'kwargs': {1}".format(level, self.get_nice_info_dict_string(info['kwargs'],
                                                                                   level + "\t",
                                                                                   overview))
            if overview['type'] == 'after':
                s += "\n{0}'result': {1}".format(level, info['result'])
            # additional elements not created by gtkmvc3 or common function calls
            overview['others'].append({})
            for key, value in info.items():
                if key in ['before', 'after', 'model', 'prop_name', 'instance', 'method_name', 'args', 'kwargs', 'result']:
                    pass
                else:
                    s += "\n{0}'{2}': {1}".format(level, info[key], key)
                    overview['others'][len(overview['others'])-1][key] = info[key]
        else:
            overview['kwargs'].append({})
            # print(info)
            # print(info['arg'])
            if isinstance(info['arg'], MetaSignalMsg):
                overview['signal'].append(info['arg'])
                s += "\n{0}'arg': MetaSignalMsg({1}".format(level,
                                                            get_nice_meta_signal_msg_tuple_string(info['arg'],
                                                                                                  level,
                                                                                                  overview))
            elif isinstance(info['arg'], ActionSignalMsg):
                overview['instance'].append(info['arg'].action_parent_m.core_element)
                overview['method_name'].append(info['arg'].action)
                overview['signal'].append(info['arg'])
                overview['kwargs'].append(info['arg'].kwargs)
                # TODO check again this stuff
                args = [info['arg'].action_parent_m.core_element, ]
                args.extend(info['arg'].kwargs.values())
                overview['args'].append(args)
                s += "\n{0}'arg': ActionSignalMsg({1}".format(level,
                                                              get_nice_action_signal_msg_tuple_string(info['arg'],
                                                                                                      level,
                                                                                                      overview))
            else:
                raise str(info)

        if overview_was_none:
            return s, overview
        else:
            return s

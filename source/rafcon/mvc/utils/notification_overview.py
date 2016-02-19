import datetime

from rafcon.statemachine.states.state import State


class NotificationOverview(dict):
    # TODO generalize

    def __init__(self, info, with_prints=False):

        self.info = info
        self.__type = 'before'
        if 'after' in info:
            self.__type = 'after'
        elif 'signal' in info:
            self.__type = 'after'
        self.with_prints = with_prints
        s, new_overview = self.get_nice_info_dict_string(info)
        self.time_stamp = datetime.datetime.now()
        self.__description = str(self.time_stamp) + "\n" + s
        self.new_overview = new_overview
        self.__overview = self.parent_state_of_notification_source(info)
        dict.__init__(self, self.__overview)
        if self.with_prints:
            print str(self)
            self.print_overview(new_overview)

    def __str__(self):
        return self.__description

    def __setitem__(self, key, value):
        if key in ['info', 'model', 'prop_name', 'instance', 'method_name', 'level']:
            dict.__setitem__(self, key, value)

    def update(self, E=None, **F):
        if E is not None:
            for key in E.keys:
                if key not in ['info', 'model', 'prop_name', 'instance', 'method_name', 'level']:
                    E.pop(key)
            dict.update(self, E)

    def print_overview(self, overview=None):
        if overview is None:
            overview = self.__overview
        info_print = ''
        info_count = 0
        for elem in overview['info']:
            info_print += "\ninfo %s: %s" % (info_count, str(elem))
            info_count += 1

        print info_print
        print "model: ", overview['model']
        print "prop_: ", overview['prop_name']
        print "insta: ", overview['instance']
        print "metho: ", overview['method_name']
        # print "level: ", overview['level']
        print "prop-: ", overview['prop_name'][-1]

    def get_all(self):
        return self.__overview

    def check_overview(self):
        overview = self.__overview
        if overview['prop_name'][-1] == 'state':
            # print "path: ", overview['instance'][-1].get_path(), "\npath: ", overview['model'][-1].state.get_path()
            assert overview['instance'][-1].get_path() == overview['model'][-1].state.get_path()
        else:
            if overview['model'][-1].parent:
                if not isinstance(overview['model'][-1].parent.state, State):  # is root_state
                    overview['model'][-1].state.get_path()
                    if self.with_prints:
                        print "Path_root: ", overview['model'][-1].state.get_path()
                else:
                    overview['model'][-1].parent.state.get_path()
                    if self.with_prints:
                        print "Path: ", overview['model'][-2].state.get_path(), "\nPath: ", \
                            overview['model'][-1].parent.state.get_path()
                    assert overview['model'][-2].state.get_path() == \
                           overview['model'][-1].parent.state.get_path().split('/')[0]

    def get_nice_info_dict_string(self, info, level='\t', overview=None):
        """ Inserts all elements of a notification info-dictionary of gtkmvc into one string and indicates levels of calls definded by 'kwargs'
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
            overview['meta_signal'].append(meta_signal_dict)

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
                overview['meta_signal'] = []

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
            s += "\n{0}'kwargs': {1}".format(level, self.get_nice_info_dict_string(info['kwargs'], level + "\t", overview))
            if overview['type'] == 'after':
                s += "\n{0}'result': {1}".format(level, info['result'])
            # additional elements not created by gtkmvc or common function calls
            overview['others'].append({})
            for key, value in info.items():
                if key in ['before', 'after', 'model', 'prop_name', 'instance', 'method_name', 'args', 'kwargs', 'result']:
                    pass
                else:
                    s += "\n{0}'{2}': {1}".format(level, info[key], key)
                    overview['others'][len(overview['others'])-1][key] = info[key]
        else:
            overview['kwargs'].append({})
            overview['meta_signal'].append(info['arg'])
            s += "\n{0}'arg': MetaSignalMsg({1}".format(level,
                                                        get_nice_meta_signal_msg_tuple_string(info['arg'], level, overview))

        if overview_was_none:
            return s, overview
        else:
            return s

    def parent_state_of_notification_source(self, info):

        if self.with_prints:
            print "----- xxxxxxx %s \n%s\n%s\n%s\n" % (self.__type, info['model'], info['prop_name'], info)

        def set_dict(info, d):
            d['model'].append(info['model'])
            d['prop_name'].append(info['prop_name'])
            d['instance'].append(info['instance'])
            d['method_name'].append(info['method_name'])
            if self.with_prints:
                print "set"

        def find_parent(info, elem):
            elem['info'].append(info)
            if 'kwargs' in info and info['kwargs']:
                if self.with_prints:
                    print 'kwargs'
                elem['level'].append('kwargs')
                set_dict(info, elem)
                if 'method_name' in info['kwargs'] and 'instance' in info['kwargs']:
                    find_parent(info['kwargs'], elem)
            elif 'info' in info and info['info']:
                if self.with_prints:
                    print 'info'
                elem['level'].append('info')
                set_dict(info, elem)
                find_parent(info['info'], elem)
                assert len(info['info']) < 2
            elif 'info' in info:
                set_dict(info, elem)
            elif 'kwargs' in info:
                set_dict(info, elem)
            else:
                if self.with_prints:
                    print 'assert'
                assert True
            return elem

        overview = find_parent(info, {'model': [], 'prop_name': [], 'instance': [], 'method_name': [], 'level': [],
                                      'info': []})

        return overview

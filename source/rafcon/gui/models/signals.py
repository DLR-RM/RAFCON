from collections import namedtuple

MetaSignalMsg = namedtuple('MetaSignalMsg', ['origin', 'change', 'affects_children', 'notification'])
MetaSignalMsg.__new__.__defaults__ = (False, None)  # Make last two parameters optional

StateTypeChangeSignalMsg = namedtuple('StateTypeChangeSignalMsg', ['new_state_m'])

Notification = namedtuple('Notification', ['model', 'prop_name', 'info'])

SelectionChangedSignalMsg = namedtuple('SelectionChangedSignalMsg', ['method_name', 'core_element_types'])

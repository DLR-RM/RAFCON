import pytest

from awesome_tool.utils import messaging
from awesome_tool.utils.messaging import MessageType
import variables_for_pytest


def test_messages():

    short_data_string = "abcdefg"
    long_data_string = "a" * 461 + "\n" + "b" * 461

    fake_root_id = "ABCDEF"
    fake_state_id = "GHIJKL"

    data_package_short_yaml = messaging.DataPackage(MessageType.YAML, fake_root_id, short_data_string, fake_state_id)
    data_package_long_yaml = messaging.DataPackage(MessageType.YAML, fake_root_id, long_data_string, fake_state_id)

    data_package_short_changed = messaging.DataPackage(MessageType.CHANGED, fake_root_id, short_data_string)
    data_package_long_changed = messaging.DataPackage(MessageType.CHANGED, fake_root_id, long_data_string)

    assert len(data_package_short_yaml.messages) == 1
    assert len(data_package_long_yaml.messages) == 3
    assert len(data_package_short_changed.messages) == 1
    assert len(data_package_long_changed.messages) == 3

    print "YAML short"
    for msg in data_package_short_yaml.messages:
        print str(msg)

    print "\nYAML long"
    for msg in data_package_long_yaml.messages:
        print str(msg)

    print "\nChanged short"
    for msg in data_package_short_changed.messages:
        print str(msg)

    print "\nChanged long"
    for msg in data_package_long_changed.messages:
        print str(msg)


if __name__ == '__main__':
    test_messages()
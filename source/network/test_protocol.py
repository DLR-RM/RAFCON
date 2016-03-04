
from protocol import Protocol, MessageType


def test_protocol():
    print "testing protocol ..."
    protocol1 = Protocol()
    assert protocol1.message_content is ""
    assert protocol1.message_type is None
    assert protocol1.sequence_number == 1

    dummy_message = "This is a dummy message"
    protocol1.message_content = dummy_message
    protocol1.message_type = MessageType.ACK
    # print protocol1.serialize()
    # protocol = protocol1.serialize().split(":")

    protocol2 = Protocol(datagram=protocol1.serialize())

    assert protocol2.message_content == dummy_message
    assert protocol2.message_type == MessageType.ACK
    assert protocol2.sequence_number == 1

if __name__ == '__main__':
    test_protocol()

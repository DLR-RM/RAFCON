import uuid
import hashlib

from awesome_tool.utils import constants
from awesome_tool.utils.network.protobuf.udp_message_pb2 import UDPMessage


def create_unique_checksum(msg):
    salt = uuid.uuid4().hex
    checksum = hashlib.md5(salt.encode() + msg.content_str.encode()).hexdigest()[:6] + ":" + salt
    if not len(checksum) == constants.CHECKSUM_LENGTH:
        raise AttributeError("MessageID has to be exact %d characters" % constants.CHECKSUM_LENGTH)
    return checksum


def check_checksum(message_id, content_str):
    checksum, salt = message_id.split(":")
    current_checksum = hashlib.md5(salt.encode() + content_str.encode()).hexdigest()[:6]
    return checksum == current_checksum


class Message:

    def __init__(self, sm_name=None, root_id=None, message=None, ack_msg=None, flag=None, proto_msg=None):
        if proto_msg:
            assert isinstance(proto_msg, UDPMessage)
            self._proto_msg = proto_msg
        else:
            if not isinstance(ack_msg, bool):
                raise AttributeError("Acknowledge flag has to be either True or False")
            if len(flag) != 3:
                raise AttributeError("Message flag has to be exact 3 chars")
            if not isinstance(sm_name, str):
                raise AttributeError("State Machine name has to be specified")
            if not isinstance(root_id, str):
                raise AttributeError("Root State ID has to be specified")
            if not isinstance(message, str):
                raise AttributeError("Message has to be specified")

            self._proto_msg = UDPMessage()
            self._proto_msg.sm_name = sm_name
            self._proto_msg.root_id = root_id
            self._proto_msg.flag = flag
            self._proto_msg.message = message
            self._proto_msg.acknowledge = ack_msg

        self._content_str = self._proto_msg.SerializeToString()

        self._message_id = create_unique_checksum(self)

    def __str__(self):
        return "%s%s" % (self.message_id, self.content_str)

    def check_checksum(self):
        return check_checksum(self.message_id, self.content_str)

    @property
    def content_str(self):
        return self._content_str

    @property
    def message_id(self):
        return self._message_id

    @property
    def sm_name(self):
        return self._proto_msg.sm_name

    @property
    def root_id(self):
        return self._proto_msg.root_id

    @property
    def message(self):
        return self._proto_msg.message

    @property
    def akg_msg(self):
        return self._proto_msg.acknowledge

    @property
    def flag(self):
        return self._proto_msg.flag

    @message.setter
    def message(self, message):
        self._proto_msg.message = message
        self._content_str = self._proto_msg.SerializeToString()
        self._message_id = create_unique_checksum(self)

    @akg_msg.setter
    def akg_msg(self, akg_msg):
        self._proto_msg.acknowledge = akg_msg
        self._content_str = self._proto_msg.SerializeToString()
        self._message_id = create_unique_checksum(self)

    @flag.setter
    def flag(self, flag):
        self._proto_msg.flag = flag
        self._content_str = self._proto_msg.SerializeToString()
        self._message_id = create_unique_checksum(self)

    @sm_name.setter
    def sm_name(self, sm_name):
        self._proto_msg.sm_name = sm_name
        self._content_str = self._proto_msg.SerializeToString()
        self._message_id = create_unique_checksum(self)

    @root_id.setter
    def root_id(self, root_id):
        self._proto_msg.root_id = root_id
        self._content_str = self._proto_msg.SerializeToString()
        self._message_id = create_unique_checksum(self)

    @staticmethod
    def parse_from_string(string):
        # ack_msg = int(string[:constants.ACK_INDICATOR_LENGTH])
        message_id = string[:constants.CHECKSUM_LENGTH]
        # flag = string[constants.ACK_INDICATOR_LENGTH+constants.CHECKSUM_LENGTH:constants.HEADER_LENGTH]
        content = string[constants.CHECKSUM_LENGTH:]

        proto_msg = UDPMessage()
        proto_msg.ParseFromString(content)

        msg = Message(proto_msg=proto_msg)
        msg._message_id = message_id
        return msg
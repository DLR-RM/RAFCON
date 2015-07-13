import uuid
import hashlib

import gobject

from awesome_tool.network.protobuf.udp_message_pb2 import UDPMessage

from awesome_tool.utils import constants
from awesome_tool.utils import log
logger = log.get_logger(__name__)


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


class Message(gobject.GObject):

    def __init__(self, sm_name=None, root_id=None, message=None, ack_msg=None, flag=None, proto_msg=None):
        self.__gobject_init__()
        if proto_msg:
            assert isinstance(proto_msg, UDPMessage)
            self._proto_msg = proto_msg
        else:
            self._proto_msg = UDPMessage()
            if not isinstance(ack_msg, bool):
                logger.warning("Acknowledge flag has to be either True or False, Default False is used.")
                self._proto_msg.acknowledge = False
            else:
                self._proto_msg.acknowledge = ack_msg

            if len(flag) != 3:
                logger.warning("Message flag has to be exact 3 chars, Default '   ' is used.")
                self._proto_msg.flag = "   "
            else:
                self._proto_msg.flag = flag

            if isinstance(sm_name, str):
                self._proto_msg.sm_name = sm_name

            if isinstance(root_id, str):
                self._proto_msg.root_id = root_id

            if not isinstance(message, str):
                logger.warning("Message has to be specified, Default 'Empty Message' is used.")
                self._proto_msg.message = "Empty Message"
            else:
                self._proto_msg.message = message

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
        return str(self._proto_msg.sm_name)

    @property
    def root_id(self):
        return str(self._proto_msg.root_id)

    @property
    def message(self):
        return str(self._proto_msg.message)

    @property
    def akg_msg(self):
        return self._proto_msg.acknowledge

    @property
    def flag(self):
        return str(self._proto_msg.flag)

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
        message_id = string[:constants.CHECKSUM_LENGTH]
        content = string[constants.CHECKSUM_LENGTH:]

        proto_msg = UDPMessage()
        proto_msg.ParseFromString(content)

        msg = Message(proto_msg=proto_msg)
        msg._message_id = message_id
        return msg

gobject.type_register(Message)
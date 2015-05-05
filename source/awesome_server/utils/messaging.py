import uuid
import hashlib

from awesome_server.utils import constants


def create_send_message(message):
    checksum = create_unique_checksum(message)
    return checksum + message


def create_unique_checksum(msg):
    salt = uuid.uuid4().hex
    checksum = hashlib.md5(salt.encode() + msg.message.encode()).hexdigest()[:6] + ":" + salt
    if not len(checksum) == constants.CHECKSUM_LENGTH:
        raise AttributeError("MessageID has to be exact %d characters" % constants.CHECKSUM_LENGTH)
    return checksum


def check_checksum(message_id, message):
    checksum, salt = message_id.split(":")
    current_checksum = hashlib.md5(salt.encode() + message.encode()).hexdigest()[:6]
    return checksum == current_checksum


class Message:

    def __init__(self, message, ack_msg, flag):
        self._message = message
        if ack_msg != 0 and ack_msg != 1:
            raise AttributeError("Acknowledge flag has to be either 0 or 1")
        self._akg_msg = ack_msg
        if len(flag) != 3:
            raise AttributeError("Message flag has to be exact 3 chars")
        self._flag = flag
        self._message_id = create_unique_checksum(self)

    def __str__(self):
        return "%d%s%s%s" % (self.akg_msg, self.message_id, self.flag, self.message)

    def check_checksum(self):
        return check_checksum(self.message_id, self.message)

    @property
    def message_id(self):
        return self._message_id

    @property
    def message(self):
        return self._message

    @property
    def akg_msg(self):
        return self._akg_msg

    @property
    def flag(self):
        return self._flag

    @message.setter
    def message(self, message):
        self._message = message
        self._message_id = create_unique_checksum(message)

    @akg_msg.setter
    def akg_msg(self, akg_msg):
        self._akg_msg = akg_msg

    @flag.setter
    def flag(self, flag):
        self._flag = flag
        self._message_id = create_unique_checksum(self)

    @staticmethod
    def parse_from_string(string):
        ack_msg = int(string[:constants.ACK_INDICATOR_LENGTH])
        message_id = string[constants.ACK_INDICATOR_LENGTH:constants.ACK_INDICATOR_LENGTH+constants.CHECKSUM_LENGTH]
        flag = string[constants.ACK_INDICATOR_LENGTH+constants.CHECKSUM_LENGTH:constants.HEADER_LENGTH]
        message = string[constants.HEADER_LENGTH:]

        msg = Message(message, ack_msg, flag)
        msg._message_id = message_id
        return msg
import uuid
import hashlib

from awesome_server.utils import constants


def create_send_message(message):
    checksum = create_unique_checksum(message)
    return checksum + message


def create_unique_checksum(message):
    salt = uuid.uuid4().hex
    checksum = hashlib.md5(salt.encode() + message.encode()).hexdigest()[:6] + ":" + salt
    if not len(checksum) == constants.CHECKSUM_LENGTH:
        raise AttributeError("MessageID has to be exact %d characters" % constants.CHECKSUM_LENGTH)
    return checksum


def check_checksum(message):
    checksum, salt = message[1:constants.HEADER_LENGTH].split(":")
    message = message[constants.HEADER_LENGTH:]
    current_checksum = hashlib.md5(salt.encode() + message.encode()).hexdigest()[:6]
    return checksum == current_checksum


class Message:

    def __init__(self, message, ack_msg):
        msg_id = create_unique_checksum(message)
        self._message_id = msg_id
        self._message = message
        self._akg_msg = ack_msg

    def __str__(self):
        return "%d%s%s" % (self.akg_msg, self.message_id, self.message)

    @property
    def message_id(self):
        return self._message_id

    @property
    def message(self):
        return self._message

    @property
    def akg_msg(self):
        return self._akg_msg

    @message.setter
    def message(self, message):
        self._message = message
        self._message_id = create_unique_checksum(message)

    @akg_msg.setter
    def akg_msg(self, akg_msg):
        self._akg_msg = akg_msg

    @staticmethod
    def parse_from_string(string):
        msg = Message(string[40:], int(string[:1]))
        msg._message_id = string[1:40]
        return msg
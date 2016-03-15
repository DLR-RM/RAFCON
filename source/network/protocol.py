from enum import Enum
import hashlib
import uuid
from config import global_network_config


class MessageType(Enum):
    ACK = 1
    STATE_ID = 2
    COMMAND = 3
    REGISTER = 4
    REGISTER_WITH_ACKNOWLEDGES = 5

STATE_EXECUTION_STATUS_SEPARATOR = "@"

sequence_number_counter = 0


def generate_sequence_number():
    """
    Generates a new sequence number
    :return: the sequence number
    """
    global sequence_number_counter
    sequence_number_counter += 1
    return sequence_number_counter


class Protocol:

    def __init__(self, message_type=None, message_content="", datagram=None):

        self.__checksum = None
        self.checksum = None
        self.__message_type = None
        self.message_type = message_type
        self.__message_content = None
        self.message_content = message_content
        self.__sequence_number = None
        self.sequence_number = None
        # for multi client identification
        self.__salt = None
        self.salt = None

        if datagram is None:
            self.sequence_number = generate_sequence_number()
            self.create_unique_checksum_and_salt()
        else:
            self.deserialize(datagram)

    def __str__(self):
        return self.serialize()

    def serialize(self):
        return str(self.create_unique_checksum_and_salt()) + ":" +\
               str(self.sequence_number) + ":" +\
               str(self.message_type.value) + ":" +\
               self.message_content

    def deserialize(self, datagram):
        try:
            checksum, salt, sequence_number, message_type, message_content = datagram.split(":")
        except Exception, e:
            raise AttributeError("datagram {0} could not be deserialized".format(datagram))

        self.checksum = checksum
        self.salt = salt
        self.sequence_number = int(sequence_number)
        self.message_type = MessageType(int(message_type))
        self.message_content = message_content
        self.check_checksum(checksum)

    @property
    def checksum(self):
        return self.__checksum

    @checksum.setter
    def checksum(self, checksum):
        if isinstance(checksum, basestring) or checksum is None:
            self.__checksum = checksum
        else:
            raise AttributeError("Cecksum must by of type basestring")

    def generate_checksum(self):
        return hashlib.md5(str(self.sequence_number) + self.salt +
                           self.message_content).hexdigest()[:global_network_config.get_config_value("HASH_LENGTH")]

    def create_unique_checksum_and_salt(self):
        if self.salt is None:
            self.salt = uuid.uuid4().hex[:global_network_config.get_config_value("SALT_LENGTH")]
        self.checksum = self.generate_checksum()
        return self.checksum + ":" + self.salt

    def check_checksum(self, checksum):
        return checksum == self.generate_checksum()

    @property
    def message_type(self):
        return self.__message_type

    @message_type.setter
    def message_type(self, message_type):
        if isinstance(message_type, MessageType) or message_type is None:
            self.__message_type = message_type
        else:
            raise AttributeError("Message type must by of type MessageType")

    @property
    def message_content(self):
        return self.__message_content

    @message_content.setter
    def message_content(self, message_content):
        if isinstance(message_content, basestring) or message_content is None:
            self.__message_content = message_content
        else:
            raise AttributeError("Content must by of type basestring")

    @property
    def sequence_number(self):
        return self.__sequence_number

    @sequence_number.setter
    def sequence_number(self, sequence_number):
        if isinstance(sequence_number, int) or sequence_number is None:
            self.__sequence_number = sequence_number
        else:
            raise AttributeError("Sequence number must by of type int")

    @property
    def salt(self):
        return self.__salt

    @salt.setter
    def salt(self, salt):
        if isinstance(salt, basestring) or salt is None:
            self.__salt = salt
        else:
            raise AttributeError("Salt must by of type basestring")




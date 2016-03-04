from enum import Enum
import hashlib
import uuid

# MessageType = Enum('MESSAGE_TYPE', 'ACK STATE_ID COMMAND')


class MessageType(Enum):
    ACK = 1
    STATE_ID = 2
    COMMAND = 3

HASH_LENGTH = 6


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

    def __init__(self, message_type=None, message_content=None, datagram=None):

        self.__checksum = None
        self.checksum = None
        self.__message_type = None
        self.message_type = message_type
        self.__message_content = None
        self.message_content = message_content
        self.__sequence_number = None
        self.sequence_number = None

        if datagram is None:
            self.sequence_number = generate_sequence_number()
        else:
            self.deserialize(datagram)

    def __str__(self):
        return self.serialize()

    def serialize(self):
        return str(self.create_unique_checksum()) + ":" +\
               str(self.sequence_number) + ":" +\
               str(self.message_type.value) + ":" +\
               self.message_content

    def deserialize(self, datagram):
        try:
            checksum, sequence_number, message_type, message_content = datagram.split(":")
        except Exception, e:
            raise AttributeError("datagram {0} could not be deserialized".format(datagram))

        self.checksum = checksum
        self.sequence_number = int(sequence_number)
        self.message_type = MessageType(int(message_type))
        self.message_content = message_content

    @property
    def checksum(self):
        return self.__checksum

    @checksum.setter
    def checksum(self, checksum):
        if isinstance(checksum, basestring) or checksum is None:
            self.__checksum = checksum
        else:
            raise AttributeError("Cecksum must by of type basestring")

    def create_unique_checksum(self):
        # salt = uuid.uuid4().hex
        return hashlib.md5(str(self.sequence_number) + self.message_content).hexdigest()[:HASH_LENGTH]

    def check_checksum(self, checksum, sequence_number):
        current_checksum = hashlib.md5(sequence_number + self.__message_content).hexdigest()[:6]
        return checksum == current_checksum

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
            raise AttributeError("Sequence number must by of type basestring")




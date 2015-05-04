import uuid
import hashlib

from awesome_tool.utils import constants


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

    def __init__(self, message, ack_msg, flag="   "):
        self._message = message
        self._akg_msg = ack_msg
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
        self._message_id = create_unique_checksum(self)

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

# from awesome_tool.utils import checksum
# from awesome_tool.utils import constants
#
# from enum import Enum
#
# from awesome_tool.utils import log
# logger = log.get_logger(__name__)
#
#
# MessageType = Enum('MESSAGE_TYPE', 'YAML CHANGED')
#
#
# class DataPackage():
#     """
#     This class stores all complete packages created for given data ready to be sent
#     :param message_type: Type of message (YAML, CHANGED)
#     :param sm_root_id: Root state ID of state machine
#     :param complete_data: complete data to be sent (str)
#     :param state_id: State ID of state to be sent (needs to be specified for YAML message)
#     """
#
#     def __init__(self, message_type, sm_root_id, complete_data, state_id=None):
#         self.message_type = message_type
#         self.messages = []
#
#         data_list = []
#         self.split_data(data_list, complete_data)
#         current_package = 1
#         for data in data_list:
#             if self.message_type == MessageType.YAML and state_id:
#                 self.messages.append(YAMLMessage(sm_root_id, data, state_id, current_package, len(data_list)))
#             elif self.message_type == MessageType.CHANGED:
#                 self.messages.append(ChangedMessage(sm_root_id, data, current_package, len(data_list)))
#             current_package += 1
#
#     def split_data(self, data_list, data):
#         if len(data) <= constants.MAXIMUM_MESSAGE_LENGTH:
#             data_list.append(data)
#         else:
#             d1 = data[:constants.MAXIMUM_MESSAGE_LENGTH]
#             d2 = data[constants.MAXIMUM_MESSAGE_LENGTH:]
#             data_list.append(d1)
#             self.split_data(data_list, d2)
#
#
# class Message(object):
#
#     def __init__(self, sm_root_id, data, package_number, total_packages):
#         self._sm_root_id = None
#         self._checksum = None
#         self._data = None
#         self._package_number = None
#         self._total_packages = None
#
#         self.sm_root_id = sm_root_id
#         self.data = data
#         self.package_number = package_number
#         self.total_packages = total_packages
#
#
# class YAMLMessage(Message):
#     """
#     Message type to store YAML messages.
#     :param sm_root_id: Root state ID of state machine
#     :param data: data from YAML-File
#     :param state_id: State ID of state to be send
#     :param package_number: Number of current data-package
#     :param total_packages: Total number of packages
#     """
#
#     init_done = False
#
#     def __init__(self, sm_root_id, data, state_id, package_number, total_packages):
#         Message.__init__(self, sm_root_id, data, package_number, total_packages)
#
#         self._state_id = None
#
#         self.state_id = state_id
#
#         self.init_done = True
#         self.update_checksum()
#
#     def __str__(self):
#         header = "%s %s %03d %03d" % (self.sm_root_id, self.state_id, self.package_number, self.total_packages)
#         return "%s %s %s" % (header, self.checksum, self.data)
#
#     def update_checksum(self):
#         if self.init_done:
#             header = "%s %s %03d %03d" % (self.sm_root_id, self.state_id, self.package_number, self.total_packages)
#             self.checksum = checksum.create_checksum(header, self.data)
#
#     @property
#     def state_id(self):
#         return self._state_id
#
#     @property
#     def package_number(self):
#         return self._package_number
#
#     @property
#     def total_packages(self):
#         return self._total_packages
#
#     @property
#     def sm_root_id(self):
#         return self._sm_root_id
#
#     @property
#     def data(self):
#         return self._data
#
#     @property
#     def checksum(self):
#         return self._checksum
#
#     @state_id.setter
#     def state_id(self, state_id):
#         if isinstance(state_id, str):
#             self._state_id = state_id
#             self.update_checksum()
#         else:
#             logger.error("state_id has to be of type 'str'")
#
#     @package_number.setter
#     def package_number(self, package_number):
#         if isinstance(package_number, int):
#             self._package_number = package_number
#             self.update_checksum()
#         else:
#             logger.error("package_number has to be of type 'int'")
#
#
#     @total_packages.setter
#     def total_packages(self, total_packages):
#         if isinstance(total_packages, int):
#             self._total_packages = total_packages
#             self.update_checksum()
#         else:
#             logger.error("total_packages has to be of type 'int'")
#
#     @sm_root_id.setter
#     def sm_root_id(self, sm_root_id):
#         if isinstance(sm_root_id, str):
#             self._sm_root_id = sm_root_id
#             self.update_checksum()
#         else:
#             logger.error("sm_root_id has to be of type 'str'")
#
#     @data.setter
#     def data(self, data):
#         if isinstance(data, str):
#             self._data = data
#             self.update_checksum()
#         else:
#             logger.error("data has to be of type 'str'")
#
#     @checksum.setter
#     def checksum(self, cs):
#         header = "%s %s %03d %03d" % (self.sm_root_id, self.state_id, self.package_number, self.total_packages)
#         if checksum.check_checksum(header, self.data, cs):
#             self._checksum = cs
#         else:
#             logger.error("Checksum not correct")
#
#
# class ChangedMessage(Message):
#     """
#     Message type to store messages indicating a change in the state machine
#     :param sm_root_id: Root state ID of state machine
#     :param data: information about the part of the state machine that changed
#     """
#
#     init_done = False
#
#     def __init__(self, sm_root_id, data, package_number, total_packages):
#         Message.__init__(self, sm_root_id, data, package_number, total_packages)
#
#         self.init_done = True
#         self.update_checksum()
#
#     def __str__(self):
#         header = "%s %03d %03d" % (self.sm_root_id, self.package_number, self.total_packages)
#         return "%s %s %s" % (header, self.checksum, self.data)
#
#     def update_checksum(self):
#         if self.init_done:
#             header = "%s %03d %03d" % (self.sm_root_id, self.package_number, self.total_packages)
#             self.checksum = checksum.create_checksum(header, self.data)
#
#     @property
#     def package_number(self):
#         return self._package_number
#
#     @property
#     def total_packages(self):
#         return self._total_packages
#
#     @property
#     def sm_root_id(self):
#         return self._sm_root_id
#
#     @property
#     def data(self):
#         return self._data
#
#     @property
#     def checksum(self):
#         return self._checksum
#
#     @package_number.setter
#     def package_number(self, package_number):
#         if isinstance(package_number, int):
#             self._package_number = package_number
#             self.update_checksum()
#         else:
#             logger.error("package_number has to be of type 'int'")
#
#
#     @total_packages.setter
#     def total_packages(self, total_packages):
#         if isinstance(total_packages, int):
#             self._total_packages = total_packages
#             self.update_checksum()
#         else:
#             logger.error("total_packages has to be of type 'int'")
#
#     @sm_root_id.setter
#     def sm_root_id(self, sm_root_id):
#         if isinstance(sm_root_id, str):
#             self._sm_root_id = sm_root_id
#             self.update_checksum()
#         else:
#             logger.error("sm_root_id has to be of type 'str'")
#
#     @data.setter
#     def data(self, data):
#         if isinstance(data, str):
#             self._data = data
#             self.update_checksum()
#         else:
#             logger.error("data has to be of type 'str'")
#
#     @checksum.setter
#     def checksum(self, cs):
#         header = "%s %03d %03d" % (self.sm_root_id, self.package_number, self.total_packages)
#         if checksum.check_checksum(header, self.data, cs):
#             self._checksum = cs
#         else:
#             logger.error("Checksum not correct")
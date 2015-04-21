from awesome_tool.utils import checksum
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class Message():

    def __init__(self, sm_root_id, data):
        self.sm_root_id = sm_root_id
        self.checksum = None
        self.data = data


class YAMLMessage(Message):
    """
    Message type to store YAML messages.
    :param sm_root_id: Root state ID of state machine
    :param data: YAML line from YAML-File
    :param state_id: State ID of state to be send
    :param line_number: Number of current line in YAML-File
    :param total_lines: Total number of lines in YAML-File
    """

    def __init__(self, sm_root_id, data, state_id, line_number, total_lines):
        Message.__init__(self, sm_root_id, data)
        self.state_id = state_id
        self.line_number = line_number
        self.total_lines = total_lines

    def __str__(self):
        header = "%s %s %06d %06d" % (self.sm_root_id, self.state_id, self.line_number, self.total_lines)
        return "%s %s %s" % (header, self.checksum, self.data)

    def update_checksum(self):
        header = "%s %s %06d %06d" % (self.sm_root_id, self.state_id, self.line_number, self.total_lines)
        self.checksum = checksum.create_checksum(header, self.data)

    @property
    def state_id(self):
        return self.state_id

    @property
    def line_number(self):
        return self.line_number

    @property
    def total_lines(self):
        return self.total_lines

    @property
    def sm_root_id(self):
        return self.sm_root_id

    @property
    def data(self):
        return self.data

    @property
    def checksum(self):
        return self.checksum

    @state_id.setter
    def state_id(self, state_id):
        self.state_id = state_id
        self.update_checksum()

    @line_number.setter
    def line_number(self, line_number):
        self.line_number = line_number
        self.update_checksum()

    @total_lines.setter
    def total_lines(self, total_lines):
        self.total_lines = total_lines
        self.update_checksum()

    @sm_root_id.setter
    def sm_root_id(self, sm_root_id):
        self.sm_root_id = sm_root_id
        self.update_checksum()

    @data.setter
    def data(self, data):
        self.data = data
        self.update_checksum()

    @checksum.setter
    def checksum(self, cs):
        header = "%s %s %06d %06d" % (self.sm_root_id, self.state_id, self.line_number, self.total_lines)
        if cs == checksum.create_checksum(header, self.data):
            self.checksum = cs
        else:
            logger.error("Checksum not correct")


class ChangedMessage(Message):
    """
    Message type to store messages indicating a change in the state machine
    :param sm_root_id: Root state ID of state machine
    :param data: information about the part of the state machine that changed
    """

    def __init__(self, sm_root_id, data):
        Message.__init__(self, sm_root_id, data)

    def __str__(self):
        return "%s %s %s" % (self.sm_root_id, self.checksum, self.data)

    def update_checksum(self):
        self.checksum = checksum.create_checksum(self.sm_root_id, self.data)

    @property
    def sm_root_id(self):
        return self.sm_root_id

    @property
    def data(self):
        return self.data

    @property
    def checksum(self):
        return self.checksum

    @sm_root_id.setter
    def sm_root_id(self, sm_root_id):
        self.sm_root_id = sm_root_id
        self.update_checksum()

    @data.setter
    def data(self, data):
        self.data = data
        self.update_checksum()

    @checksum.setter
    def checksum(self, cs):
        if cs == checksum.create_checksum(self.sm_root_id, self.data):
            self.checksum = cs
        else:
            logger.error("Checksum not correct")
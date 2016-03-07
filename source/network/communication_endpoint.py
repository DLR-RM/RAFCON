import threading
from time import gmtime, strftime
import time
from config import global_config
from protocol import Protocol, MessageType
import collections

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from multiprocessing import Queue
from rafcon.utils import log
logger = log.get_logger(__name__)


MAX_TIME_WAITING_FOR_ACKNOWLEDGEMENTS = global_config.get_config_value("MAX_TIME_WAITING_FOR_ACKNOWLEDGEMENTS")
CHECK_ACKNOWLEDGEMENTS_THREAD_MAX_WAIT_TIME = 0.1
BURST_NUMBER = global_config.get_config_value("BURST_NUMBER")
TIME_BETWEEN_BURSTS = global_config.get_config_value("TIME_BETWEEN_BURSTS")


class CommunicationEndpoint(DatagramProtocol):

    def __init__(self):
        self._new_message_cv = threading.Condition()

        self.datagram_received_function = self.print_message
        # primitive data types are thread safe in python, thus they are not secured by a lock
        self._messages_to_be_acknowledged = {}
        self._messages_to_be_acknowledged_timeout = {}
        self._acknowledge_messages_address_couples = []
        self._registered_endpoints = {}
        self._registered_endpoints_for_acknowledgements = []
        self.number_of_dropped_messages = 0
        self._message_history = collections.deque(maxlen=global_config.get_config_value("HISTORY_LENGTH"))
        # this is only to speed up memory access times
        self._message_history_dictionary = {}

        self.check_acknowledgements_thread = threading.Thread(target=self.check_acknowledgements)

    def check_acknowledgements(self):

        while True:
            next_message = None
            logger.debug("Check_acknowledgements thread looping")

            # get new message thread safe
            self._new_message_cv.acquire()
            while True:
                self._new_message_cv.wait(CHECK_ACKNOWLEDGEMENTS_THREAD_MAX_WAIT_TIME)
                if len(self._acknowledge_messages_address_couples) > 0 or len(self._messages_to_be_acknowledged) > 0:
                    break

            if len(self._acknowledge_messages_address_couples) > 0:
                next_message, address = self._acknowledge_messages_address_couples.pop()
                assert isinstance(next_message, Protocol)
            self._new_message_cv.release()

            # delete received messages from _messages_to_be_acknowledged
            if next_message is not None:
                if next_message.message_content in self._messages_to_be_acknowledged.iterkeys():
                    logger.debug("Message {0} was acknowledged successfully".format(str(next_message)))
                    del self._messages_to_be_acknowledged[next_message.message_content]
                    del self._messages_to_be_acknowledged_timeout[next_message.message_content]
                else:
                    logger.warn("Message {0} was acknowledged that was sent by another endpoint"
                                " or was already dropped".format(str(next_message)))

            # check messages for timeout
            # logger.debug("check_acknowledgements checking for messages timeout")
            messages_to_be_droped = []
            for key, message in self._messages_to_be_acknowledged.iteritems():
                self._messages_to_be_acknowledged_timeout[key] += CHECK_ACKNOWLEDGEMENTS_THREAD_MAX_WAIT_TIME
                if self._messages_to_be_acknowledged_timeout[key] > MAX_TIME_WAITING_FOR_ACKNOWLEDGEMENTS:
                    messages_to_be_droped.append(key)

            for key in messages_to_be_droped:
                # this is not the right strategy:
                # logger.warn("Message {0} dropped because of timeout".format(self._messages_to_be_acknowledged[key]))
                # del self._messages_to_be_acknowledged[key]
                # del self._messages_to_be_acknowledged_timeout[key]
                # self.number_of_dropped_messages += 1
                logger.warn("Message {0} is going to be resent as no acknowledge was received yet".
                            format(self._messages_to_be_acknowledged[key]))
                self.send_message_acknowledged(self._messages_to_be_acknowledged[key])

    def datagramReceived(self, datagram, address):

        # parsing message
        try:
            protocol = Protocol(datagram=datagram)
        except Exception, e:
            import traceback
            logger.error("Received message could not be deserialized: {0} {1}".format(e.message, traceback.format_exc()))

        # throwing away messages that were received before
        if protocol.checksum not in self._message_history_dictionary.iterkeys():
            # check if ringbuffer is already full
            if len(self._message_history) == self._message_history.maxlen:
                oldest_history_element = self._message_history.popleft()
                del self._message_history_dictionary[protocol.checksum]
            self._message_history.append(protocol.checksum)
            self._message_history_dictionary[protocol.checksum] = protocol

            # custom function
            self.datagram_received_function(datagram, address)
        else:
            # logger.info("Message was already received!")
            return

        # registering endpoints
        if protocol.message_type is MessageType.REGISTER \
                or protocol.message_type is MessageType.REGISTER_WITH_ACKNOWLEDGES:
            if address not in self._registered_endpoints:
                logger.info("Endpoint with address {0} registered".format(str(address)))
                self._registered_endpoints[address] = strftime("%Y-%m-%d %H:%M:%S", gmtime())
                if protocol.message_type is MessageType.REGISTER_WITH_ACKNOWLEDGES:
                    self._registered_endpoints_for_acknowledgements.append(address)

        # add the acknowledge message to its responsible list and wake up the thread checking the acknowledgements
        if protocol.message_type is MessageType.ACK:
            self._new_message_cv.acquire()
            self._acknowledge_messages_address_couples.append((protocol, address))
            self._new_message_cv.notify()
            self._new_message_cv.release()
        else:  # acknowledge message if endpoint registered for acknowledgements
            if address in self._registered_endpoints_for_acknowledgements:
                ack_message = Protocol(MessageType.ACK, protocol.checksum)
                self.send_message_non_acknowledged(ack_message, address)
            else:
                pass

    def send_message_non_acknowledged(self, message, address=None):
        for i in range(0, BURST_NUMBER):
            self.transport.write(message.serialize(), address)
            time.sleep(TIME_BETWEEN_BURSTS)

    def send_message_acknowledged(self, message):
        assert isinstance(message, Protocol)
        self._messages_to_be_acknowledged[message.checksum] = message
        self._messages_to_be_acknowledged_timeout[message.checksum] = 0
        self.send_message_non_acknowledged(message)

    def get_transport(self):
        return self.transport

    def messages_to_be_acknowledged_pending(self):
        if len(self._messages_to_be_acknowledged) > 0:
            return True
        else:
            return False

    @staticmethod
    def print_message(message, address):
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))
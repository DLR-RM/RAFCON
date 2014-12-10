"""
.. module:: execution_history
   :platform: Unix, Windows
   :synopsis: A module for the history of one thread during state machine execution

.. moduleauthor:: Sebastian Brunner


"""


class ExecutionHistory():

    """A class for the history of one thread during state machine execution

    Each state machine thread has its own execution history

    :ivar _history_transitions: all transitions that where used in the past
    :ivar _history_timestamps: all timestamps for all past state entries
    :ivar _crash_reason: an optional crash reason, if the thread was unintentionally closed
    :ivar _last_outcome: holds the last outcome of the last state (the last state can be determined by the history
                        of the transitions)


    """

    def __init__(self):

        self._history_transitions = None
        self._history_timestamps = None
        self._crash_reason = None
        self._last_outcome = None
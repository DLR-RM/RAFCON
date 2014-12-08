"""
.. module:: outcome
   :platform: Unix, Windows
   :synopsis: A module for representing an outcome of a state

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import numpy


class Outcome(Observable):

    """A class for representing an outcome of a state

    It inherits from Observable to make a change of its fields observable.

    :ivar _outcome_id: the id of the outcome, must be unique on one hierarchy level
    :ivar _name: the human readable name of the outcome

    """

    def __init__(self, outcome_id=None, name=None):

        Observable.__init__(self)

        self._outcome_id = outcome_id
        self._name = name


#########################################################################
# Properties for all class field that must be observed by the gtkmvc
#########################################################################

    @property
    def outcome_id(self):
        """Property for the _outcome_id field

        """
        return self._outcome_id

    @outcome_id.setter
    @Observable.observed
    def outcome_id(self, outcome_id):
        if not isinstance(outcome_id, numpy.uint32):
            raise TypeError("outcome_id must be of type numpy.uint32")

        self._outcome_id = outcome_id

    @property
    def name(self):
        """Property for the _name field

        """
        return self._name

    @name.setter
    @Observable.observed
    def name(self, name):
        if not isinstance(name, str):
            raise TypeError("name must be of type str")

        self._name = name


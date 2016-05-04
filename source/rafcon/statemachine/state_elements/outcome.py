"""
.. module:: outcome
   :platform: Unix, Windows
   :synopsis: A module to represent an outcome in the state machine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable

from rafcon.statemachine.state_elements.state_element import StateElement
from rafcon.utils import log
logger = log.get_logger(__name__)


class Outcome(StateElement):
    """A class for representing an outcome of a state

    It inherits from Observable to make a change of its fields observable.

    As the name of an outcome can be changes without modifying the transitions the primary key of an outcome is its
    id and not its name.

    :ivar int outcome_id: the id of the outcome, must be unique on one hierarchy level
    :ivar str name: the human readable name of the outcome
    :ivar rafcon.statemachine.states.state.State parent: reference to the parent state
    """

    yaml_tag = u'!Outcome'

    _outcome_id = None
    _name = None

    def __init__(self, outcome_id=None, name=None, parent=None):
        super(Outcome, self).__init__()

        if not isinstance(outcome_id, int):
            raise TypeError("outcome_id must be of type int")
        self._outcome_id = outcome_id

        self.name = name

        # Checks for validity
        self.parent = parent

        # logger.debug("Outcome with name %s and id %s initialized" % (self.name, self.outcome_id))

    def __str__(self):
        return "Outcome '{0}' [{1}]".format(self.name, self.outcome_id)

    def __copy__(self):
        return self.__class__(self._outcome_id, self._name, None)

    __deepcopy__ = __copy__

    @classmethod
    def from_dict(cls, dictionary):
        return Outcome(dictionary['outcome_id'], dictionary['name'])

    @staticmethod
    def state_element_to_dict(state_element):
        return {
            'outcome_id': state_element.outcome_id,
            'name': state_element.name
        }

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        outcome_id = dict_representation['outcome_id']
        name = dict_representation['name']
        return Outcome(outcome_id, name)


#########################################################################
# Properties for all class field that must be observed by the gtkmvc
#########################################################################

    @property
    def outcome_id(self):
        """Property for the _outcome_id field

        """
        return self._outcome_id

    @property
    def name(self):
        """Property for the _name field
        """
        return self._name

    @name.setter
    @Observable.observed
    def name(self, name):
        if not isinstance(name, basestring):
            raise TypeError("name must be of type str")

        if len(name) < 1:
            raise ValueError("Name cannot be empty")

        self._change_property_with_validity_check('_name', name)


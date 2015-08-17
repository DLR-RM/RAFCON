"""
.. module:: outcome
   :platform: Unix, Windows
   :synopsis: A module for representing an outcome of a state

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import yaml

from awesome_tool.utils import log
logger = log.get_logger(__name__)


class Outcome(Observable, yaml.YAMLObject):

    """A class for representing an outcome of a state

    It inherits from Observable to make a change of its fields observable.

    As the name of an outcome can be changes without modifying the transitions the primary key of an outcome is its
    id and not its name.

    :ivar outcome_id: the id of the outcome, must be unique on one hierarchy level
    :ivar name: the human readable name of the outcome
    :ivar parent: reference to the parent state
    """

    yaml_tag = u'!Outcome'

    def __init__(self, outcome_id=None, name=None, parent=None):
        Observable.__init__(self)

        # Prevents validity checks by parent before all parameters are set
        self._parent = None

        self._outcome_id = None
        self.outcome_id = outcome_id

        self._name = None
        self.name = name

        # Checks for validity
        self.parent = parent

        logger.debug("Outcome with name %s and id %s initialized" % (self.name, self.outcome_id))

    def __str__(self):
        return "Outcome '{0}' [{1}]".format(self.name, self.outcome_id)

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'outcome_id': data.outcome_id,
            'name': data.name,
        }
        node = dumper.represent_mapping(u'!Outcome', dict_representation)
        return node

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

    @outcome_id.setter
    @Observable.observed
    def outcome_id(self, outcome_id):
        if not isinstance(outcome_id, int):
            raise TypeError("outcome_id must be of type int")

        self.__change_property_with_validity_check('_outcome_id', outcome_id)

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

        self.__change_property_with_validity_check('_name', name)

    @property
    def parent(self):
        return self._parent

    @parent.setter
    @Observable.observed
    def parent(self, parent):
        if parent is not None:
            from awesome_tool.statemachine.states.state import State
            assert isinstance(parent, State)

        self.__change_property_with_validity_check('_parent', parent)

    def __change_property_with_validity_check(self, property_name, value):
        """Helper method to change a property and reset it if the validity check fails

        :param str property_name: The name of the property to be changed, e.g. '_name'
        :param value: The new desired value for this property
        """
        assert isinstance(property_name, str)
        old_value = getattr(self, property_name)
        setattr(self, property_name, value)

        valid, message = self._check_validity()
        if not valid:
            setattr(self, property_name, old_value)
            if property_name == '_parent':
                raise ValueError("Outcome invalid: {0}".format(message))
            raise ValueError("The outcome's '{0}' could not be changed: {1}".format(property_name[1:], message))

    def _check_validity(self):
        """Checks the validity of the Outcome properties

        The validity of the Outcome can only be checked by the parent, to see if there are other outcomes with the
        same name or id. Thus, the existence of a parent and a check function must be ensured and this function be
        queried.

        :return: (True, str message) if valid, (False, str reason) else
        """
        if not self.parent:
            return True, "no parent"
        if not hasattr(self.parent, 'check_child_validity') or \
                not callable(getattr(self.parent, 'check_child_validity')):
            return True, "no parental check"
        return self.parent.check_child_validity(self)



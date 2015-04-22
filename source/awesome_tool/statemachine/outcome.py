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
    :ivar check_name: a function handle to a function checking the integrity of the outcome name. Used in the setter
            of the name

    """

    yaml_tag = u'!Outcome'

    def __init__(self, outcome_id=None, name=None, check_name_func_handle=None):

        Observable.__init__(self)

        self._outcome_id = None
        self.outcome_id = outcome_id

        self.check_name = check_name_func_handle

        self._name = None
        self.name = name

        logger.debug("Outcome with name %s and id %s initialized" % (self.name, self.outcome_id))

    def __str__(self):
        return "Outcome - outcome_id: %s, name: %s" % (self._outcome_id, self._name)

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

        if self.check_name is not None:
            self._name = self.check_name(name, self)
        else:
            self._name = name


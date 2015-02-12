"""
.. module:: validity_checker
   :platform: Unix, Windows
   :synopsis: A module implementing a generic validity check for a statemachine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable

from utils import log
logger = log.get_logger(__name__)
import vc_strategy
import vc_implementations


class ValidityChecker(Observable):

    """A strategy pattern class for checking the validity of a container state

    It inherits from Observable to make a change of its fields observable.

    :ivar _strategy: defines the strategy of the validity checker
    :ivar _vcInstance: instance of the validity checker incarnating a specific strategy

    """

    def __init__(self, strategy=vc_strategy.Strategy.DEFAULT):
        Observable.__init__(self)
        if not isinstance(strategy, vc_strategy.Strategy):
            raise TypeError("No valid strategy given for ValidityChecker")
        self._strategy = strategy
        self._strategy_instance = vc_strategy.VCStrategy()

        if strategy is vc_strategy.Strategy.DEFAULT:
            self._strategy_instance = vc_implementations.LightVC()
        elif strategy is vc_strategy.Strategy.NONE:
            self._strategy_instance = vc_implementations.DummyVC()
        elif strategy is vc_strategy.Strategy.LIGHT:
            self._strategy_instance = vc_implementations.LightVC()
        else:
            self._strategy_instance = vc_implementations.AggressiveVC()

    def check(self, c_state):
        """The interface function to trigger the check of a container state

        :param c_state: the container class to  be checked

        """
        #TODO: deferred import: can this be somehow solved otherwise???
        import statemachine.states.container_state as smcs

        if not isinstance(c_state, smcs.ContainerState):
            raise TypeError("Check() can only be applied onto ContainerStates")
        logger.debug("Check the validity of the container state with id %s " % str(c_state.state_id()))
        return self._strategy_instance.check(c_state)
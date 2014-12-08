"""
.. module:: transition
   :platform: Unix, Windows
   :synopsis: A module defining different validity check class for checking the correctness of a container state

.. moduleauthor::

"""

import vc_strategy


class DummyVC(vc_strategy.VCStrategy):

    def __init__(self):
        pass

    #Note: Use deferred import to check cState for being instacne of ContainerState
    #from statemachine.container_state import ContainerState
    def check(self, cState):
        #TODO: implement
        return True


class LightVC(vc_strategy.VCStrategy):

    def __init__(self):
        pass

    def check(self, cState):
        #TODO: implement
        return True


class AggressiveVC(vc_strategy.VCStrategy):

    def __init__(self):
        pass

    def check(self, cState):
        #TODO: implement
        return True



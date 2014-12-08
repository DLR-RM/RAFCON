"""
.. module:: vc_implementation
   :platform: Unix, Windows
   :synopsis: A module defining different validity check class for checking the correctness of a container state

.. moduleauthor::

"""

import vc_strategy


class DummyVC(vc_strategy.VCStrategy):

    def __init__(self):
        vc_strategy.VCStrategy.__init__(self)
        pass

    #Note: Use deferred import to check cState for being instacne of ContainerState
    #from statemachine.container_state import ContainerState
    def check(self, container_state):
        #TODO: implement
        return True


class LightVC(vc_strategy.VCStrategy):

    def __init__(self):
        vc_strategy.VCStrategy.__init__(self)

    def check(self, container_state):
        #TODO: implement
        return True


class AggressiveVC(vc_strategy.VCStrategy):

    def __init__(self):
        vc_strategy.VCStrategy.__init__(self)

    def check(self, container_state):
        #TODO: implement
        return True
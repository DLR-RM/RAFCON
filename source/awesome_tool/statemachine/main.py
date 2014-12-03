
from state import State
from container_state import ContainerState


if __name__ == '__main__':
    state1 = State()
    state2 = State()
    state2.start()

    state3 = ContainerState()
    state3.run()
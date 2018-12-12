import time
import pykeyboard
from future.utils import string_types

keyboard = pykeyboard.PyKeyboard()


def press_key(characters, duration=0.05):
    print characters
    assert all([isinstance(character, (int, string_types)) for character in characters])
    assert isinstance(duration, (int, float))
    for character in characters:
        print("press_key: ", character)
        keyboard.press_key(character=character)
    print("for {0} seconds".format(duration))
    time.sleep(duration)
    for character in characters:
        print("release_key: ", character)
        keyboard.release_key(character=character)

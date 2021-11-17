import time
import pykeyboard

keyboard = pykeyboard.PyKeyboard()


def press_key(characters, duration=0.05):
    assert all([isinstance(character, (int, str)) for character in characters])
    assert isinstance(duration, (int, float))
    for character in characters:
        print("press_key: ", character)
        keyboard.press_key(character=character)
    print("for {0} seconds".format(duration))
    time.sleep(duration)
    for character in characters:
        print("release_key: ", character)
        keyboard.release_key(character=character)

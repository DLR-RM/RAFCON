import pytest

configs = [("core", "config.yaml"), ("gui", "gui_config.yaml"), ("runtime", "runtime_config.yaml")]
config_contents = {}


def pytest_configure(config):
    # register additional markers "core", "gui", "share_elements" and "network"
    config.addinivalue_line("markers", "core: mark test as being located in the core folder")
    config.addinivalue_line("markers", "gui: mark test as being located in the gui folder")
    config.addinivalue_line("markers", "share_elements: mark test as being located in the share_elements folder")
    config.addinivalue_line("markers", "network: mark test as being located in the network folder")
    config.addinivalue_line("markers", "user_input: mark test as imitating user inputs using pyuserinput")


def pytest_unconfigure(config):
    clean_temp_test_directory()


def pytest_collection_modifyitems(items):
    for item in items:
        if item.nodeid.startswith("tests/core/"):
            item.add_marker(pytest.mark.core)
        elif item.nodeid.startswith("tests/gui/"):
            item.add_marker(pytest.mark.gui)
        elif item.nodeid.startswith("tests/share_elements/"):
            item.add_marker(pytest.mark.share_elements)
        elif item.nodeid.startswith("tests/network/"):
            item.add_marker(pytest.mark.network)


def clean_temp_test_directory():
    import shutil
    import os
    from tests import utils as testing_utils
    test_temp_path = testing_utils.RAFCON_TEMP_PATH_TEST_BASE
    try:
        shutil.rmtree(test_temp_path)
        os.mkdir(test_temp_path)
    except OSError:
        pass

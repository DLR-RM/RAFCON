
def pytest_unconfigure(config):
    clean_temp_test_directory()


def clean_temp_test_directory():
    import shutil
    import testing_utils
    test_temp_path = testing_utils.RAFCON_TEMP_PATH_TEST_BASE
    shutil.rmtree(test_temp_path)

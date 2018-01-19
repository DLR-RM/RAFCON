
# Unfortunately this approach does not work to make sure to initialize the gui singletons from a gui thread
# Problem: after executing conftest.py all modules are re-imported
# and thus all variables (incl. singletons) are reinitialized
# def pytest_configure(config):
#     if any(x in str(config.invocation_dir) for x in ["gui", "share_elements", "widget", "network"]):
#         import testing_utils
#         testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
#         try:
#             # do nothing, just open gui and close it afterwards
#             pass
#         except:
#             raise
#         finally:
#             testing_utils.close_gui()
#             testing_utils.shutdown_environment()


def pytest_unconfigure(config):
    clean_temp_test_directory()


def clean_temp_test_directory():
    import shutil
    import os
    import testing_utils
    test_temp_path = testing_utils.RAFCON_TEMP_PATH_TEST_BASE
    try:
        shutil.rmtree(test_temp_path)
        os.mkdir(test_temp_path)
    except OSError:
        pass

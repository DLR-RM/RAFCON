import testing_utils


def test_thread_observer_creation_list(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    try:
        # do nothing, just open gui and close it afterwards
        assert True
    except:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0)

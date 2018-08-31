import testing_utils
from core.test_i18n import use_locale


def test_gtk_translation(caplog, monkeypatch):
    with use_locale("de_DE.UTF-8", monkeypatch):
        # i18n.setup_l10n() is called inside run_gui()
        testing_utils.run_gui()

        try:

            import rafcon.gui.singleton
            main_window_controller = rafcon.gui.singleton.main_window_controller
            gvm_controller = main_window_controller.get_controller('global_variable_manager_ctrl')
            gvm_view = gvm_controller.view
            remove_button = gvm_view["delete_global_variable_button"]

            print "Found text in label is: ", _("Remove"), remove_button.get_label()
            assert _("Remove") == "Entfernen" == remove_button.get_label()

        finally:
            testing_utils.close_gui()
            testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    import pytest
    pytest.main([__file__])

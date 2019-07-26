from __future__ import print_function
from tests.core.test_i18n import use_locale, create_mo_files


def test_gtk_translation(gui, monkeypatch):
    create_mo_files()

    with use_locale("de_DE.UTF-8", monkeypatch):
        gui.restart()

        main_window_controller = gui.singletons.main_window_controller
        gvm_controller = main_window_controller.get_controller('global_variable_manager_ctrl')
        gvm_view = gvm_controller.view
        remove_button = gvm_view["delete_global_variable_button"]

        print("Found text in label is: ", _("Remove"), remove_button.get_label())
        assert _("Remove") == "Entfernen" == remove_button.get_label()


if __name__ == '__main__':
    import pytest
    pytest.main([__file__])

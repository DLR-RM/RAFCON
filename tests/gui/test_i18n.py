import os

from rafcon.utils import i18n

import testing_utils


def test_gtk_translation(caplog):
    testing_utils.run_gui()

    try:
        os.environ["LANG"] = "de_DE.UTF-8"
        i18n.setup_l10n()

        import rafcon.gui.singleton
        main_window_controller = rafcon.gui.singleton.main_window_controller
        gvm_controller = main_window_controller.get_controller('global_variable_manager_ctrl')
        gvm_view = gvm_controller.view
        remove_button = gvm_view["delete_global_variable_button"]

        assert _("Remove") == "Entfernen" == remove_button.get_label()

    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)

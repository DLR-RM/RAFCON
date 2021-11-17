from tests.core.test_i18n import use_locale, create_mo_files
from tests import utils as testing_utils

from rafcon.utils import i18n, log
logger = log.get_logger(__name__)


def test_gtk_translation(gui, monkeypatch):
    create_mo_files()

    # target_locale = "de_DE.UTF-8"
    # on debian systems the german utf locale is: "de_DE.utf8"
    target_locale = "de_DE.utf8"
    if testing_utils.check_if_locale_exists(target_locale):
        logger.info("Execute locale test as locale was found on system")
        with use_locale("de_DE.UTF-8", monkeypatch):
            gui.restart()

            main_window_controller = gui.singletons.main_window_controller
            gvm_controller = main_window_controller.get_controller('global_variable_manager_ctrl')
            gvm_view = gvm_controller.view
            remove_button = gvm_view["delete_global_variable_button"]

            print("Found text in label is: ", _("Remove"), remove_button.get_label())
            assert _("Remove") == "Entfernen" == remove_button.get_label()
    else:
        logger.info("Locale test is not executed as locale was NOT found on system")


if __name__ == '__main__':
    import pytest
    pytest.main([__file__])

import os

from rafcon.utils import i18n

import testing_utils


def test_invalid_locale_setting(caplog):
    os.environ["LANG"] = "de"
    i18n.setup_l10n()

    testing_utils.assert_logger_warnings_and_errors(caplog=caplog, expected_warnings=1)


def test_basic_string_translation(caplog):
    os.environ["LANG"] = "de_DE.UTF-8"
    i18n.setup_l10n()

    assert _("Remove") == "Entfernen"

    testing_utils.assert_logger_warnings_and_errors(caplog=caplog)

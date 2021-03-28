from builtins import range
import os
from contextlib import contextmanager

from rafcon.utils import i18n, log

from tests import utils as testing_utils

logger = log.get_logger(__name__)


def create_mo_files():
    curdir = os.path.abspath(os.curdir)
    while "setup.py" not in os.listdir(os.curdir):
        os.chdir("..")
    i18n.create_mo_files(logger)
    os.chdir(curdir)


@contextmanager
def use_locale(locale, monkeypatch):
    create_mo_files()

    # See https://www.gnu.org/software/gettext/manual/html_node/Locale-Environment-Variables.html
    locale_env_vars = ["LANGUAGE", "LC_ALL", "LC_MESSAGES", "LANG"]
    for env_var in locale_env_vars:
        monkeypatch.setenv(env_var, locale)
    try:
        yield
    finally:
        for _ in range(len(locale_env_vars)):
            monkeypatch.undo()


def test_invalid_locale_setting(caplog, monkeypatch):
    create_mo_files()

    with use_locale("invalid", monkeypatch):
        i18n.setup_l10n(logger)

    testing_utils.assert_logger_warnings_and_errors(caplog=caplog, expected_warnings=1)


def _test_basic_string_translation(caplog, monkeypatch):
    with use_locale("de_DE.UTF-8", monkeypatch):
        i18n.setup_l10n(logger)
        assert _("Remove") == "Entfernen"

    testing_utils.assert_logger_warnings_and_errors(caplog=caplog)


if __name__ == '__main__':
    import pytest
    pytest.main([__file__])

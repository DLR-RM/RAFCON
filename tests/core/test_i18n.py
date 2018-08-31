from contextlib import contextmanager

from rafcon.utils import i18n

import testing_utils


@contextmanager
def use_locale(locale, monkeypatch):
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
    with use_locale("invalid", monkeypatch):
        i18n.setup_l10n()

    testing_utils.assert_logger_warnings_and_errors(caplog=caplog, expected_warnings=1)


def test_basic_string_translation(caplog, monkeypatch):
    with use_locale("de_DE.UTF-8", monkeypatch):
        i18n.setup_l10n()
        assert _("Remove") == "Entfernen"

    testing_utils.assert_logger_warnings_and_errors(caplog=caplog)


if __name__ == '__main__':
    import pytest
    pytest.main([__file__])

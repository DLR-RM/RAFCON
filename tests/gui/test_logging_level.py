import logging


def test_shown_levels_set_logger_level(gui):
    """The lowest level shown in the logging console is the level of the rafcon logger, also for the terminal"""
    from rafcon.gui.config import global_gui_config
    rafcon_logger = logging.getLogger("rafcon")

    def show(**levels):
        for level, shown in levels.items():
            gui(global_gui_config.set_config_value, "LOGGING_SHOW_{}".format(level.upper()), shown)

    show(verbose=False, debug=False, info=True, warning=True)
    assert rafcon_logger.level == logging.INFO
    show(debug=True)
    assert rafcon_logger.level == logging.DEBUG
    show(verbose=True)
    assert rafcon_logger.level == logging.VERBOSE
    show(verbose=False, debug=False, info=False)
    assert rafcon_logger.level == logging.WARNING
    show(warning=False)
    assert rafcon_logger.level == logging.ERROR  # errors are always logged
    show(info=True, warning=True)
    assert rafcon_logger.level == logging.INFO

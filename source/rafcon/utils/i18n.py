import locale
import gettext

import gtk

_ = gettext.gettext


def setup_l10n():
    locale.setlocale(locale.LC_ALL, '')
    gettext.bindtextdomain('rafcon', 'locale')
    gettext.textdomain('rafcon')

    for module in (gettext, gtk.glade):
        module.bindtextdomain('rafcon', 'locale')
        module.textdomain('rafcon')

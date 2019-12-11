# Copyright (C) 2016-2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>

import os
from os.path import join, splitext
import locale
import gettext

from pkg_resources import resource_filename


def setup_l10n(logger=None):
    """Setup RAFCON for localization

    Specify the directory, where the translation files (*.mo) can be found (rafcon/locale/) and set localization domain
    ("rafcon").

    :param logger: which logger to use for printing (either logging.log or distutils.log)
    """
    try:
        locale.setlocale(locale.LC_ALL, '')
    except locale.Error as e:
        logger and logger.warning("Cannot setup translations: {}".format(e))

    localedir = resource_filename('rafcon', 'locale')

    # Install gettext globally: Allows to use _("my string") without further imports
    gettext.install('rafcon', localedir)

    # Required for glade and the GtkBuilder
    locale.bindtextdomain('rafcon', localedir)
    locale.textdomain('rafcon')


def create_mo_files(logger):
    assert "setup.py" in os.listdir(os.curdir)
    import subprocess
    domain = "rafcon"
    rel_localedir = join('source', 'rafcon', 'locale')
    localedir = join(os.curdir, rel_localedir)
    # Assert that we are in the root directory of the RAFCON repository
    po_files = [po_file
                for po_file in next(os.walk(localedir))[2]
                if splitext(po_file)[1] == '.po']
    for po_file in po_files:
        po_path = join(localedir, po_file)
        lang, extension = splitext(po_file)
        mo_file = domain + '.mo'
        mo_dir = join(localedir, lang, 'LC_MESSAGES')
        mo_path = join(mo_dir, mo_file)
        try:
            os.makedirs(mo_dir)
        except os.error:  # already exists
            pass
        msgfmt_cmd = 'msgfmt -o {} {}'.format(mo_path, po_path)
        result = subprocess.call(msgfmt_cmd, shell=True)
        if result != 0:  # Compilation successful
            logger.warning("Could not compile translation '{}'. RAFCON will not be available in this "
                        "language.".format(lang))
        else:
            logger.info("Sucessfully compiled '{}' translation file".format(lang))

#!/usr/bin/env python3.6

# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import sys
import sass
import pathlib
import distutils.log


PRE_BUILD_SCRIPT_DIRECTORY = pathlib.Path(__file__).parent.resolve()
SASS_PATH = str(PRE_BUILD_SCRIPT_DIRECTORY / "source" / "rafcon" / "share" / "themes" / "RAFCON" / "sass")
CSS_PATH = str(PRE_BUILD_SCRIPT_DIRECTORY / "source" / "rafcon" / "share" / "themes" / "RAFCON" / "gtk-3.0")

class BuildMOFiles:
    description = "Create/update mo translation files"
    user_options = []

    def initialize_options(self):
        pass  # must be overridden

    def finalize_options(self):
        pass  # must be overridden

    def run(self):
        import importlib
        sys.path.insert(0, "./source")
        i18n = importlib.import_module("rafcon.utils.i18n")
        i18n.create_mo_files(distutils.log)
        sys.path.pop()


if __name__ == '__main__':
    # generate mo files
    distutils.log.set_verbosity(distutils.log.INFO)
    mo_builder = BuildMOFiles()
    mo_builder.run()

    # generate css from sass files
    sass.compile(dirname=(SASS_PATH, CSS_PATH))

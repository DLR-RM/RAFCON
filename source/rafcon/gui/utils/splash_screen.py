# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import os
from gi.repository import Gtk
from gi.repository import GdkPixbuf
import random
from pkg_resources import resource_filename, resource_listdir

import rafcon.gui
from rafcon.utils import log

logger = log.get_logger(__name__)


class SplashScreen(Gtk.Window):

    def __init__(self, width=530, height=350, contains_image=False):
        # init Gtk.Window with type popup
        super(SplashScreen, self).__init__(type=Gtk.WindowType.POPUP)

        # index for the image rotator
        self.image_index = 0

        # Set the title to rafcon so it is detectable in taskbars
        # set width and height to parameter values and position the window in the center
        self.set_title('RAFCON')
        self.set_default_size(width, height)
        self.set_position(Gtk.WindowPosition.CENTER)

        main_vbox = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        self.add(main_vbox)
        self.image = Gtk.Image()
        # If an img path was defined, create a gtk img and fill it from a pixelbuffer which is created from the
        # set file path
        if contains_image:
            main_vbox.pack_start(self.image, True, True, 0)

        # add label to display text, the text can be changed by the text() method.
        # Align it in the middle of the gtk window
        self.label = Gtk.Label(label="")
        self.label.set_xalign(0.5)
        self.label.set_yalign(0.5)
        main_vbox.pack_start(self.label, False, True, 10)
        main_vbox.set_spacing(0)

        if not os.getenv("RAFCON_START_MINIMIZED", False):
            self.show_all()

    def set_text(self, text):
        logger.info(text)
        self.label.set_text(text)
        # include this to give more time to watch
        # import time
        # time.sleep(1)
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)
        return

    def load_image(self, image_path):
        if image_path:
            pixbuf = GdkPixbuf.Pixbuf.new_from_file_at_size(image_path, self.get_size()[0] - 50, self.get_size()[1] - 50)
            self.image.set_from_pixbuf(pixbuf)
            while Gtk.events_pending():
                Gtk.main_iteration_do(False)
        else:
            logger.debug("Splash screen image path is None")

    def rotate_image(self, random_=True):
        images = []
        for image_filename in resource_listdir(rafcon.gui.__name__, os.path.join("assets", "splashscreens")):
            images.append(resource_filename(rafcon.gui.__name__, os.path.join("assets", "splashscreens", image_filename)))

        # if random mode is specified, choose a picture out of the target folder. Else switch through the pictures
        if random_:
            image_path = images[int(random.uniform(0.0, len(images)))]
        else:
            if self.image_index >= len(images):
                self.image_index = 0
            image_path = images[self.image_index]
            self.image_index += 1

        self.load_image(image_path)


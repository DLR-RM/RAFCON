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
from gi.repository import GLib
from gi.repository import GdkPixbuf
import random

import rafcon.gui
from rafcon.gui.design_config import global_design_config, is_custom_design_enabled
from rafcon.utils import log

logger = log.get_logger(__name__)


def _pump_main_context():
    """Processes pending main loop events (GTK4 replacement for Gtk.events_pending loops)"""
    context = GLib.MainContext.default()
    while context.pending():
        context.iteration(False)


class SplashScreen(Gtk.Window):

    def __init__(self, width=530, height=350, contains_image=False):
        # GTK4 has no popup window type or window positioning; an undecorated window is used
        super(SplashScreen, self).__init__()
        self.set_decorated(False)

        # index for the image rotator
        self.image_index = 0

        # remember the requested size; GTK4 windows cannot be queried before mapping
        self.width = width
        self.height = height

        # Set the title to rafcon so it is detectable in taskbars
        self.set_title('RAFCON')
        self.set_default_size(width, height)

        main_vbox = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        self.set_child(main_vbox)
        self.image = Gtk.Image()
        # If an img path was defined, create a gtk img and fill it from a pixelbuffer which is created from the
        # set file path
        if contains_image:
            self.image.set_vexpand(True)
            main_vbox.append(self.image)

        if global_design_config.get_config_value("SPLASH_SCREEN_SHOW_TEXT", True):
            # add label to display text, the text can be changed by the text() method.
            # Align it in the middle of the gtk window
            self.label = Gtk.Label(label="")
            self.label.set_xalign(0.5)
            self.label.set_yalign(0.5)
            main_vbox.append(self.label)
            main_vbox.set_spacing(0)
            label_height = global_design_config.get_config_value("SPLASH_SCREEN_LABEL_HEIGHT", 0)
            self.label.set_size_request(-1, label_height)

        if not os.getenv("RAFCON_START_MINIMIZED", False):
            self.present()

    def set_text(self, text):
        if not global_design_config.get_config_value("SPLASH_SCREEN_SHOW_TEXT", True):
            return
        logger.info(text)
        self.label.set_text(text)
        _pump_main_context()
        return

    def load_image(self, image_path):
        if image_path:
            horizontal_spacing = global_design_config.get_config_value("SPLASH_SCREEN_HORIZONTAL_SPACING", 50)
            pixbuf = GdkPixbuf.Pixbuf.new_from_file_at_size(image_path, self.width - horizontal_spacing,
                                                            self.height - horizontal_spacing)
            self.image.set_from_pixbuf(pixbuf)
            _pump_main_context()
        else:
            logger.debug("Splash screen image path is None")

    def get_images(self):
        images = list()
        splash_screen_path = global_design_config.get_config_value("SPLASH_SCREEN_FOLDER")
        for image_filename in os.listdir(splash_screen_path):
            images.append(os.path.join(splash_screen_path, image_filename))
        return images

    def rotate_image(self, random_=True):
        images = self.get_images()
        # if random mode is specified, choose a picture out of the target folder. Else switch through the pictures
        if random_:
            image_path = images[int(random.uniform(0.0, len(images)))]
        else:
            if self.image_index >= len(images):
                self.image_index = 0
            image_path = images[self.image_index]
            self.image_index += 1

        self.load_image(image_path)

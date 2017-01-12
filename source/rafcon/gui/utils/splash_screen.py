import gtk
import time
import os
import random
import rafcon

from rafcon.utils import log

logger = log.get_logger(__name__)


class SplashScreen(gtk.Window):

    def __init__(self, width=800, height=200, contains_image=False):
        # init gtk.Window with type popup
        super(SplashScreen, self).__init__(type=gtk.WINDOW_POPUP)

        # index for the image rotator
        self.img_index = 0

        # Set the title to rafcon so it is detectable in taskbars
        # set width and height to parameter values and position the window in the center
        self.set_title('RAFCON')
        self.set_default_size(width, height)
        self.set_position(gtk.WIN_POS_CENTER)

        main_vbox = gtk.VBox(False, 1)
        self.add(main_vbox)
        self.image = gtk.Image()
        # If an img path was defined, create a gtk img and fill it from a pixelbuffer which is created from the
        # set file path
        if contains_image:
            main_vbox.pack_start(self.image, True, True)

        # add label to display text, the text can be changed by the text() method.
        # Align it in the middle of the gtk window
        self.lbl = gtk.Label("")
        self.lbl.set_alignment(0.5, 0.5)
        main_vbox.pack_start(self.lbl, True, True)

        self.show_all()

    def text(self, text):
        self.lbl.set_text(text)
        while gtk.events_pending():
            gtk.main_iteration_do(True)
        return

    def load_image(self, path):
        if path:
            pixbuf = gtk.gdk.pixbuf_new_from_file_at_size(path, self.get_size()[0]-50, self.get_size()[1]-50)
            self.image.set_from_pixbuf(pixbuf)
            while gtk.events_pending():
                gtk.main_iteration_do(True)
        else:
            logger.debug("Splash screen image path is None")

    def rotate(self, rand=True, img_folder=None):

        if not img_folder:
            # TODO: Replace this with an actual folder
            img_folder = os.path.join(rafcon.__path__[0], 'gui', 'utils', 'images')

        # get content of the specified img folder
        paths = os.listdir(img_folder)
        # if random mode is specified, choose a picture out of the target folder. Else switch through the pictures
        if rand:
            path = paths[int(random.uniform(0.0, len(paths)))]
        else:
            if self.img_index >= len(paths):
                self.img_index = 0
            path = paths[self.img_index]
            self.img_index += 1

        self.load_image(img_folder + "/" + path)


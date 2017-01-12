import gtk
import time

class SplashScreen(object):

    def __init__(self, width=800, height=200, img_path=None, debug=False):
        self.debug = debug

        # Set up generic window as a popup. Set the title to rafcon so it is detectable in taskbars
        # set width and height to parameter values and position the window in the center
        self.window = gtk.Window(gtk.WINDOW_POPUP)
        self.window.set_title('RAFCON')
        self.window.set_default_size(width, height)
        self.window.set_position(gtk.WIN_POS_CENTER)

        main_vbox = gtk.VBox(False, 1)
        self.window.add(main_vbox)

        # If an img path was defined, create a gtk img and fill it from a pixelbuffer which is created from the
        # set file path
        if img_path:
            self.image = gtk.Image()

            self.pixbuf = gtk.gdk.pixbuf_new_from_file_at_size(img_path, width-50, height-50)
            self.image.set_from_pixbuf(self.pixbuf)
            main_vbox.pack_start(self.image, True, True)

        # add label to display text, the text can be changed by the text() method.
        # Align it in the middle of the gtk window
        self.lbl = gtk.Label("")
        self.lbl.set_alignment(0.5, 0.5)
        main_vbox.pack_start(self.lbl, True, True)

        self.window.show_all()

    def text(self, text):
        self.lbl.set_text(text)
        if self.debug:
            time.sleep(1)
        while gtk.events_pending():
            gtk.main_iteration_do(True)
        return
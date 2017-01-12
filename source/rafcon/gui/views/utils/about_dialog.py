# coding=utf-8
import gtk
import rafcon



class MyAboutDialog(gtk.AboutDialog):
    def __init__(self):
        gtk.AboutDialog.__init__(self)

        self.set_program_name("RAFCON")
        self.set_version(rafcon.__version__)
        self.set_authors(("Rico Belder", "Sebastian Brunner", "Franz Steinmetz", "Michael Vilzmann", "Lukas Becker",
                          "Annika Wollschläger", "Benno Voggenreiter", "Matthias Büttner", "Mahmoud Akl"))
        # TODO: set copyright/license
        # self.set_copyright("Copyright: DLR")
        # self.set_license("Copyright: DLR")
        self.set_website("https://rmintra01.robotic.dlr.de/wiki/RAFCON")

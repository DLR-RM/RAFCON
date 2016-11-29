import gtk


class MyAboutDialog(gtk.AboutDialog):
    def __init__(self):
        gtk.AboutDialog.__init__(self)

        self.set_program_name("RAFCON")
        # TODO: retrieve version number from somewhere
        self.set_version("0.2.0")
        self.set_authors(("Rico Belder", "Sebastian Brunner", "Franz Steinmetz"))
        self.set_copyright("Copyright: DLR")
        self.set_website("https://rmintra01.robotic.dlr.de/wiki/RAFCON")

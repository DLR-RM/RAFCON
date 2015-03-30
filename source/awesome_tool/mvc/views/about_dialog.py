import gtk


class MyAboutDialog(gtk.AboutDialog):

    def __init__(self):
        gtk.AboutDialog.__init__(self)

        self.set_program_name("Awesome Tool")
        self.set_version("x.x.x")
        self.set_authors(("Rico Belder", "Sebastian Brunner", "Franz Steinmetz"))
        self.set_copyright("Copyright: DLR")
        self.set_website("https://rmintra01.robotic.dlr.de/wiki/Awesome_tool")
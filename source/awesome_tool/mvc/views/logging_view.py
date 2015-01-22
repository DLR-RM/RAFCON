import gtk
from gtkmvc import View


class LoggingView(View):
    top = 'main_frame'

    #===============================================================
    def __init__(self):
        View.__init__(self)

        window = gtk.Window()
        window.resize(width=500, height=200)
        window.set_property("title", "Logging")

        # create textview
        self.textview = None
        self.textview = gtk.TextView()
        self.textview.set_property('editable', False)
        #self.textview.get_buffer().create_tag("dead_color", foreground="gray")
        self.textview.get_buffer().create_tag("default", font="Monospace 10")
        self.textview.get_buffer().create_tag("set_warning_color", foreground="blue")
        self.textview.get_buffer().create_tag("set_error_color"  , foreground="red")
        self.textview.get_buffer().create_tag("set_debug_color"  , foreground="green")
        self.textview.get_buffer().create_tag("set_info_color"   , foreground="orange")

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.add(self.textview)
        window.add(scrollable)
        window.set_property("visible", True)
        window.show_all()

        self['main_window'] = window
        self['scrollable'] = scrollable

    def apply_tag(self, name):
        self.textview.get_buffer().apply_tag_by_name(name,
                                                     self.textview.get_buffer().get_start_iter(),
                                                     self.textview.get_buffer().get_end_iter())

    def print_debug(self, text):
        self.print_add(text, "set_warning_color")

    def print_error(self, text):
        self.print_add(text, "set_error_color")

    def get_buffer(self):
        return self.textview.get_buffer()

    def set_text(self, text):
        self.textview.get_buffer().set_text(text)

    def print_add(self, text_to_add, use_tag=None):
        text_to_add += "\n"
        self.print_push(text_to_add, use_tag)

    def print_push(self, text_to_push, use_tag=None):
        text_buf = self.textview.get_buffer()
        if use_tag:
            text = text_buf.insert_with_tags_by_name(text_buf.get_end_iter(), text_to_push, use_tag)
        else:
            text = text_buf.insert(text_buf.get_end_iter(), text_to_push)
        self.textview.scroll_to_iter(text_buf.get_end_iter(), 0.0, use_align=True , yalign=0.0)

    def print_clean(self, keep_lines=0):
        text_buf = self.textview.get_buffer()
        text = ""
        self.textview.get_buffer().set_text(text)
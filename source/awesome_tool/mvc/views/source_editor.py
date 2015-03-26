import gtk

try:
    import gtksourceview2
except:
    print "NO python-module gtksourceview2 found!"

from gtkmvc import View
import awesome_tool.statemachine.config as config


#TODO: comment

class SourceEditorView(View):
    top = 'source_editor_frame'

    #===============================================================
    def __init__(self):
        View.__init__(self)

        #window = gtk.Window()
        #window.resize(width=550, height=500)
        vbox = gtk.VBox()
        #window.add(vbox)
        hbox = gtk.HBox()
        apply_button = gtk.Button("Apply")
        cancel_button = gtk.Button("Cancel")
        hbox.pack_start(apply_button)
        hbox.pack_end(cancel_button)

        editor_frame = gtk.Frame()
        vbox.pack_start(editor_frame, expand=True, fill=True)
        vbox.pack_end(hbox, expand=False, fill=False)
        l = gtk.Label()
        vbox.pack_start(l, expand=False, fill=False)

        # create textview
        self.textview = None
        try:
            lm = gtksourceview2.LanguageManager()
            sm = gtksourceview2.StyleSchemeManager()
            if 'python' in lm.get_language_ids():
                self.textview = gtksourceview2.View(gtksourceview2.Buffer())
                b = self.textview.get_buffer()
                b.set_language(lm.get_language('python'))
                b.set_highlight_syntax(True)
                if sm.get_scheme(config.global_config.get_config_value("SOURCE_EDITOR_STYLE")):
                    b.set_style_scheme(sm.get_scheme(config.global_config.get_config_value("SOURCE_EDITOR_STYLE")))
                else:
                    b.set_style_scheme(sm.get_scheme('classic'))
                self.textview.set_tab_width(4)
                self.textview.set_insert_spaces_instead_of_tabs(True)
                self.textview.set_show_line_numbers(True)
                self.textview.set_auto_indent(True)
                self.textview.set_highlight_current_line(True)
                self.textview.set_mark_category_pixbuf('INSTRUCTION',
                                                       editor_frame.render_icon(gtk.STOCK_GO_FORWARD, gtk.ICON_SIZE_MENU))
                self.using_source_view = True
            else:
                self.textview = gtk.TextView()
                self.using_source_view = False
        except NameError:
            self.textview = gtk.TextView()
            self.using_source_view = False

        self.textview.get_buffer().create_tag("dead_color", foreground="gray")
        self.textview.get_buffer().create_tag("default", font="Monospace 10")
        self.textview.get_buffer().connect('changed', self.code_changed)

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.add(self.textview)
        editor_frame.add(scrollable)

        #window.set_property("visible", True)
        #window.show_all()

        #self['main_window'] = window
        self['source_editor_frame'] = vbox
        self['apply_button'] = apply_button
        self['cancel_button'] = cancel_button

    def apply_tag(self, name):
        self.textview.get_buffer().apply_tag_by_name(name,
                                                     self.textview.get_buffer().get_start_iter(),
                                                     self.textview.get_buffer().get_end_iter())

    def code_changed(self, source):
        self.apply_tag('default')

    def get_buffer(self):
        return self.textview.get_buffer()

    def set_text(self, text):
        self.textview.get_buffer().set_text(text)

    def set_enabled(self, on):
        if on:
            self.apply_tag('default')
        else:
            self.apply_tag('dead_color')
        self.textview.set_property('editable', on)

import gtk
from gtkmvc import View

from rafcon.mvc.config import global_gui_config
from rafcon.mvc.utils import constants
from rafcon.mvc import gui_helper
from rafcon.utils import log

logger = log.get_logger(__name__)

try:
    import gtksourceview2
except ImportError:
    print "Python module 'gtksourceview2' not found!"


class EditorView(View):
    top = 'editor_frame'

    def __init__(self, name='SOURCE EDITOR', language='idl', editor_style="SOURCE_EDITOR_STYLE"):
        View.__init__(self)

        vbox = gtk.VBox()

        source_label = gui_helper.create_label_with_text_and_spacing(name, letter_spacing=constants.LETTER_SPACING_1PT)
        source_label.set_alignment(0.0, 0.5)
        source_box = gtk.EventBox()
        source_box.set_name(name.replace(' ', '_').lower() + '_label_wrapper')
        source_box.set_border_width(constants.BORDER_WIDTH_TEXTVIEW)
        source_box.add(source_label)
        self.event_box = source_box

        title_viewport = gtk.Viewport()
        title_viewport.set_name(name.replace(' ', '_').lower() + "_title_wrapper")
        title_viewport.add(source_box)
        title_viewport.show_all()

        editor_frame = gtk.Frame()
        vbox.pack_start(title_viewport, False, True, 0)
        vbox.pack_start(editor_frame, expand=True, fill=True)

        # create textview
        self.textview = None
        try:
            language_manager = gtksourceview2.LanguageManager()
            style_scheme_manager = gtksourceview2.StyleSchemeManager()
            if language in language_manager.get_language_ids():
                self.textview = gtksourceview2.View(gtksourceview2.Buffer())
                b = self.textview.get_buffer()
                b.set_language(language_manager.get_language(language))
                b.set_highlight_syntax(True)
                if global_gui_config.get_config_value(editor_style) and \
                        style_scheme_manager.get_scheme(global_gui_config.get_config_value(editor_style)):
                    b.set_style_scheme(style_scheme_manager.get_scheme(
                        global_gui_config.get_config_value(editor_style))
                    )
                else:
                    logger.debug("Edit style '{}' is not supported take "
                                 "default: 'classic'".format(global_gui_config.get_config_value(editor_style)))
                    b.set_style_scheme(style_scheme_manager.get_scheme('classic'))
                self.textview.set_mark_category_pixbuf('INSTRUCTION',
                                                       editor_frame.render_icon(gtk.STOCK_GO_FORWARD,
                                                                                gtk.ICON_SIZE_MENU))
                self.using_source_view = True
            else:
                logger.debug("Chosen language '{}' is not supported initiate simple TextView.".format(language))
                self.textview = gtk.TextView()
                self.using_source_view = False
        except NameError:
            self.textview = gtk.TextView()
            self.using_source_view = False

        self.textview.get_buffer().create_tag("deactivated", foreground="gray")
        self.textview.get_buffer().create_tag("default", font="Monospace 10")
        self.textview.get_buffer().connect('changed', self.code_changed)

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.add(self.textview)
        editor_frame.add(scrollable)
        self.scrollable = scrollable

        self['editor_frame'] = vbox

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
            self.apply_tag('deactivated')
        self.textview.set_property('editable', on)

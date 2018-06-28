# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
from gtkmvc import View

import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui.config import global_gui_config
from rafcon.gui.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)

try:
    import gtksourceview2
except ImportError:
    logger.warning("Python module 'gtksourceview2' not found!")


class EditorView(View):

    def __init__(self, name='SOURCE EDITOR', language='idl', editor_style="SOURCE_EDITOR_STYLE", run_with_spacer=False):
        View.__init__(self)

        vbox = gtk.VBox()

        # create title view port widget
        source_label = gui_helper_label.create_label_with_text_and_spacing(name,
                                                                           letter_spacing=constants.LETTER_SPACING_1PT)
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

        # prepare frame for the text editor
        editor_frame = gtk.Frame()

        # create textview/sourceview2
        self.textview = None
        self.style_scheme = None
        self.language = language
        self.editor_style = editor_style
        try:
            self.language_manager = gtksourceview2.LanguageManager()
            if language in self.language_manager.get_language_ids():

                self.textview = gtksourceview2.View(self.new_buffer())
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

        self.while_in_set_enabled = False
        self.register()

        # wrap text view with scroller window
        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.add(self.textview)
        self.scrollable = scrollable

        # wrap scroller window with gtk.Frame for proper viewing
        editor_frame.add(scrollable)

        # fill top widget vbox with title view port, source view and text view within
        vbox.pack_start(title_viewport, False, True, 0)
        self.spacer_frame = None
        if run_with_spacer:
            # with spacer a gtk.Frame object is used as spacer and its is with the source view in one hbox
            hbox_frame = gtk.HBox()
            self.spacer_frame = gtk.Frame()
            hbox_frame.pack_end(self.spacer_frame, expand=False, fill=False)
            hbox_frame.pack_start(editor_frame, expand=True, fill=True)
            vbox.pack_start(hbox_frame, expand=True, fill=True)
        else:
            vbox.pack_start(editor_frame, expand=True, fill=True)

        self['editor_frame'] = vbox
        self.top = 'editor_frame'

    def new_buffer(self):
        style_scheme_manager = gtksourceview2.StyleSchemeManager()
        b = gtksourceview2.Buffer()
        b.set_language(self.language_manager.get_language(self.language))
        b.set_highlight_syntax(True)

        user_editor_style = global_gui_config.get_config_value(self.editor_style, "classic")
        scheme = style_scheme_manager.get_scheme(user_editor_style)
        if scheme:
            self.style_scheme = scheme
        else:
            logger.debug("The editor style '{}' is not supported. Using the default 'classic'".format(
                user_editor_style))
            self.style_scheme = style_scheme_manager.get_scheme('classic')
        b.set_style_scheme(self.style_scheme)
        return b

    def register(self):
        self.textview.get_buffer().create_tag("deactivated", foreground="gray")
        self.textview.get_buffer().create_tag("default", font="Monospace 10")
        self.textview.get_buffer().connect('changed', self.code_changed)

    def apply_tag(self, name):
        text_buffer = self.get_buffer()
        text_buffer.apply_tag_by_name(name, text_buffer.get_start_iter(), text_buffer.get_end_iter())

    def code_changed(self, source):
        self.apply_tag('default')

    def get_buffer(self):
        return self.textview.get_buffer()

    def get_text(self):
        return self.get_buffer().get_text(self.get_buffer().get_start_iter(), self.get_buffer().get_end_iter())

    def set_text(self, text):
        """ The method insert text into the text buffer of the text view and preserves the cursor location.

        :param str text: which is insert into the text buffer.
        :return:
        """
        line_number, line_offset = self.get_cursor_position()
        self.get_buffer().set_text(text)
        self.set_cursor_position(line_number, line_offset)

    def set_enabled(self, on, text=None):
        """ Set the default input or deactivated (disabled) style scheme

        The method triggers the signal 'changed' by using set_text. Therefore, the method use the while_in_set_enabled
        flag to make activities of the method observable. If a method trigger this method and was triggered by a
        changed-signal this flag is supposed to avoid recursive calls.

        :param bool on: enable flag.
        :param str text: optional text to insert.
        :return:
        """
        self.while_in_set_enabled = True

        # Apply color scheme by set text 'workaround' (with current buffer source)
        self.set_text(self.get_text()) if text is None else self.set_text(text)

        if on:
            self.textview.set_editable(True)
            self.apply_tag('default')
        else:
            self.apply_tag('deactivated')
        self.textview.set_editable(on)
        self.while_in_set_enabled = False

    def scroll_to_cursor_onscreen(self):
        self.textview.scroll_mark_onscreen(self.get_buffer().get_insert())

    def get_cursor_position(self):
        text_buffer = self.get_buffer()
        p_iter = text_buffer.get_iter_at_offset(text_buffer.props.cursor_position)
        return p_iter.get_line(), p_iter.get_line_offset()

    def set_cursor_position(self, line_number, line_offset):
        text_buffer = self.get_buffer()
        new_p_iter = text_buffer.get_iter_at_line(line_number)
        if new_p_iter.get_chars_in_line() >= line_offset:
            new_p_iter = text_buffer.get_iter_at_line_offset(line_number, line_offset)
        else:
            logger.debug("Line has not enough chars {0} {1}".format((line_number, line_offset), new_p_iter.get_chars_in_line()))
        if new_p_iter.is_cursor_position():
            result = text_buffer.place_cursor(new_p_iter)
        else:
            if not (line_offset == 0 and new_p_iter.get_chars_in_line() == 0):
                logger.debug("Line and offset is no cursor position line: {0} offset: {1} line length: {2}"
                             "".format(line_number, line_offset, new_p_iter.get_chars_in_line()))
            result = False

        self.scroll_to_cursor_onscreen()
        return result

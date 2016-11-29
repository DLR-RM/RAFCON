import gtk

from rafcon.gui.utils import constants
from rafcon.gui.views.utils.editor import EditorView, gtksourceview2


class SourceEditorView(EditorView):

    def __init__(self):
        super(SourceEditorView, self).__init__(name='SOURCE EDITOR', language='python', editor_style="SOURCE_EDITOR_STYLE")

        try:
            if isinstance(self.textview, gtksourceview2.View):
                self.textview.set_tab_width(4)
                self.textview.set_insert_spaces_instead_of_tabs(True)
                self.textview.set_show_line_numbers(True)
                self.textview.set_auto_indent(True)
                self.textview.set_highlight_current_line(True)
        except NameError:
            pass
        hbox = gtk.HBox()
        pylint_check_button = gtk.CheckButton("Pylint-Check")
        pylint_check_button.set_focus_on_click(False)

        open_external_button = gtk.ToggleButton("Open externally")
        open_external_button.set_focus_on_click(False)
        open_external_button.set_size_request(120, constants.BUTTON_MIN_HEIGHT)

        apply_button = gtk.Button("Apply")
        apply_button.set_focus_on_click(False)
        apply_button.set_size_request(constants.BUTTON_MIN_WIDTH, constants.BUTTON_MIN_HEIGHT)

        cancel_button = gtk.Button("Cancel")
        cancel_button.set_focus_on_click(False)
        cancel_button.set_size_request(constants.BUTTON_MIN_WIDTH, constants.BUTTON_MIN_HEIGHT)

        hbox.pack_start(pylint_check_button, False, False, constants.PADDING)
        hbox.pack_end(open_external_button, False, True, constants.PADDING)
        hbox.pack_end(cancel_button, False, True, constants.PADDING)
        hbox.pack_end(apply_button, False, True, constants.PADDING)
        hbox.set_border_width(constants.BORDER_WIDTH)
        self['editor_frame'].pack_start(hbox, expand=False, fill=True)
        self['pylint_check_button'] = pylint_check_button
        self['apply_button'] = apply_button
        self['open_external_button'] = open_external_button
        self['cancel_button'] = cancel_button


from rafcon.mvc.views.editor import EditorView, gtksourceview2


class DescriptionEditorView(EditorView):

    def __init__(self):
        super(DescriptionEditorView, self).__init__(name='DESCRIPTION', language='idl',
                                                    editor_style="DESCRIPTION_EDITOR_STYLE")

        try:
            if isinstance(self.textview, gtksourceview2.View):
                self.textview.set_wrap_mode(True)
                self.textview.set_tab_width(4)
                self.textview.set_insert_spaces_instead_of_tabs(True)
                self.textview.set_show_line_numbers(False)
                self.textview.set_auto_indent(True)
                self.textview.set_highlight_current_line(True)
        except NameError:
            pass

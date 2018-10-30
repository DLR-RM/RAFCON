# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import GtkSource
from rafcon.gui.views.utils.editor import EditorView


class DescriptionEditorView(EditorView):

    def __init__(self):
        super(DescriptionEditorView, self).__init__(name='DESCRIPTION', language='idl',
                                                    editor_style="SOURCE_EDITOR_STYLE")

        try:
            if isinstance(self.textview, GtkSource.View):
                self.textview.set_wrap_mode(True)
                self.textview.set_tab_width(4)
                self.textview.set_insert_spaces_instead_of_tabs(True)
                self.textview.set_show_line_numbers(False)
                self.textview.set_auto_indent(True)
                self.textview.set_highlight_current_line(True)
                b = self.textview.get_buffer()
                b.set_highlight_syntax(False)
        except NameError:
            pass

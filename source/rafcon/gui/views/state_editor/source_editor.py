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

from gi.repository import Gtk
from gi.repository import GtkSource

from rafcon.gui.utils import constants
from rafcon.gui.helpers import label
from rafcon.gui.views.utils.editor import EditorView


class SourceEditorView(EditorView):

    def __init__(self):
        super(SourceEditorView, self).__init__(name='SOURCE EDITOR', language='python',
                                               editor_style="SOURCE_EDITOR_STYLE", run_with_spacer=False)

        try:
            if isinstance(self.textview, GtkSource.View):
                self.textview.set_tab_width(4)
                self.textview.set_insert_spaces_instead_of_tabs(True)
                self.textview.set_show_line_numbers(True)
                self.textview.set_auto_indent(True)
                self.textview.set_highlight_current_line(True)
        except NameError:
            pass
        hbox = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)
        hbox.get_style_context().add_class("widget-toolbar")
        pylint_check_button = Gtk.CheckButton(label="Validate")
        Gtk.Widget.set_focus_on_click(pylint_check_button, True)
        pylint_check_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        pylint_check_button.get_style_context().add_class("secondary")

        open_external_button = Gtk.ToggleButton(label="Open externally")
        Gtk.Widget.set_focus_on_click(open_external_button, True)
        open_external_button.set_border_width(constants.BUTTON_BORDER_WIDTH)

        apply_button = Gtk.Button(label="Apply")
        Gtk.Widget.set_focus_on_click(apply_button, True)
        apply_button.set_border_width(constants.BUTTON_BORDER_WIDTH)

        cancel_button = Gtk.Button(label="Reset")
        Gtk.Widget.set_focus_on_click(cancel_button, True)
        cancel_button.set_border_width(constants.BUTTON_BORDER_WIDTH)

        hbox.pack_start(pylint_check_button, False, False, 0)
        hbox.pack_end(open_external_button, False, True, 0)
        hbox.pack_end(cancel_button, False, True, 0)
        hbox.pack_end(apply_button, False, True, 0)

        label.ellipsize_labels_recursively(hbox)

        self['editor_frame'].pack_start(hbox, expand=False, fill=True, padding=0)
        self['pylint_check_button'] = pylint_check_button
        self['apply_button'] = apply_button
        self['open_external_button'] = open_external_button
        self['cancel_button'] = cancel_button

        # TODO find the properties where the the next three values can be read from
        # value is an assumption because its respective property is not found till now
        self.line_numbers_width = 35
        # this value is from the main window glade file and respective right_bar_container width request
        self.tab_width = 53
        # value is an assumption because its respective property is not found till now
        self.source_view_character_size = 8
        # observe key press events to adapt pane position
        # Note: -> changed is not used because it is creating glib segfaults
        if self.spacer_frame is not None:
            self.textview.connect("key-press-event", self.on_text_view_event)

    def on_draw(self, widget, event):
        # Gtk TODO: Produces warnings and does not work properly
        # Warnings should not be related to the flickering of the spacer_frame
        # https://developer.gnome.org/gtk3/stable/ch26s02.html#id-1.6.3.4.11
        if self.run_with_spacer:
            right_bar_width_of_all = self.button_container_min_width + self.tab_width + self.line_numbers_width
            if right_bar_width_of_all > event.clip_extents()[2]:
                spacer_width = right_bar_width_of_all - event.clip_extents()[2]
                self.spacer_frame.set_size_request(width=spacer_width, height=-1)
            else:
                self.spacer_frame.set_size_request(width=-1, height=-1)

    @property
    def button_container_min_width(self):
        return self['pylint_check_button'].get_allocation().width + self['apply_button'].get_size_request()[0] + \
               self['open_external_button'].get_allocation().width + self['cancel_button'].get_size_request()[0]

    def on_text_view_event(self, *args):
        self.pane_position_check()

    def pane_position_check(self):
        """ Update right bar pane position if needed

        Checks calculates if the cursor is still visible and updates the pane position if it is close to not be seen.
        In case of an un-docked right-bar this method does nothing.
        :return:
        """
        text_buffer = self.get_buffer()

        # not needed if the right side bar is un-docked
        from rafcon.gui.singleton import main_window_controller

        if main_window_controller is None or main_window_controller.view is None:
            return
        from rafcon.gui.runtime_config import global_runtime_config
        if global_runtime_config.get_config_value('RIGHT_BAR_WINDOW_UNDOCKED'):
            return

        # move the pane left if the cursor is to far right and the pane position is less then 440 from its max position
        button_container_min_width = self.button_container_min_width
        width_of_all = button_container_min_width + self.tab_width
        text_view_width = button_container_min_width - self.line_numbers_width
        min_line_string_length = float(button_container_min_width)/float(self.source_view_character_size)

        current_pane_pos = main_window_controller.view['right_h_pane'].get_property('position')
        max_position = main_window_controller.view['right_h_pane'].get_property('max_position')
        pane_rel_pos = main_window_controller.view['right_h_pane'].get_property('max_position') - current_pane_pos
        if pane_rel_pos >= width_of_all + self.line_numbers_width:
            pass
        else:
            cursor_line_offset = text_buffer.get_iter_at_offset(text_buffer.props.cursor_position).get_line_offset()
            needed_rel_pos = text_view_width/min_line_string_length*cursor_line_offset \
                             + self.tab_width + self.line_numbers_width
            needed_rel_pos = min(width_of_all, needed_rel_pos)
            if pane_rel_pos >= needed_rel_pos:
                pass
            else:
                main_window_controller.view['right_h_pane'].set_property('position', max_position - needed_rel_pos)
                spacer_width = int(width_of_all + self.line_numbers_width - needed_rel_pos)
                self.spacer_frame.set_size_request(width=spacer_width, height=-1)

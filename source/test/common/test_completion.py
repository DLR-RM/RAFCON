__author__ = 'sietsopur'
#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#
#  Copyright 2015 Erik Daguerre <fallenwolf@meddlesomewolf.com>
#  Special thanks to @zeroSteiner for help debugging and suggestions
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of the  nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import keyword
import re

import gtk
import gtksourceview2
import gobject
# import gobject._gobject as __gobject__gobject


class CustomCompletionProvider(gtksourceview2.CompletionWords):
    """
    This is a custom Completion Provider
    In this instance, it will do 2 things;
    1) always provide Hello World! (Not ideal but an option so its in the example)
    2) Utilizes the Gtk.TextIter from the TextBuffer to determine if there is a jinja
    example of '{{ custom.' if so it will provide you with the options of foo and bar.
    if select it will insert foo }} or bar }}, completing your syntax
    PLEASE NOTE the GtkTextIter Logic and regex are really rough and should be adjusted and tuned
    to fit your requirements
    # Implement the Completion Provider
    # http://stackoverflow.com/questions/32611820/implementing-gobject-interfaces-in-python
    # https://gist.github.com/andialbrecht/4463278 (Python example implementing TreeModel)
    # https://developer.gnome.org/gtk3/stable/GtkTreeModel.html (Gtk TreeModel interface specification)
    # A special thank you to @zeroSteiner
    """

    def __init__(self):
        gobject.GObject.__init__(self)

    def do_get_name(self):
        print "get name"
        return 'PythonKeywords'

    def do_get_activation(self):
        print "get activation"
        return gtksourceview2.COMPLETION_ACTIVATION_USER_REQUESTED

    def do_match(self, context):
        print "do_match"
        return True

    def do_get_start_iter(self, context):
        print "do_get start iter"
        return context.get_iter()

    def do_activate_proposal(self, proposal, iter):
        return True
    # apparently interface methods MUST be prefixed with do_

    # def do_match(self, context):
    #     # this should evaluate the context to determine if this completion
    #     # provider is applicable, for debugging always return True
    #     return True

    def do_populate(self, context):
        print "do_populate"
        proposals = [
            gtksourceview2.CompletionItem(label='Hello World!', text='Hello World!')  # always proposed
        ]
        for compl in keyword.kwlist:
            print compl
            proposals.append(gtksourceview2.CompletionItem(compl, compl))

        # found difference in Gtk Versions
        end_iter = context.get_iter()
        if not isinstance(end_iter, gtk.TextIter):
            _, end_iter = context.get_iter()

        if end_iter:
            buf = end_iter.get_buffer()
            mov_iter = end_iter.copy()
            if mov_iter.backward_search('{{', gtk.TextSearchFlags.VISIBLE_ONLY):
                mov_iter, _ = mov_iter.backward_search('{{', gtk.TextSearchFlags.VISIBLE_ONLY)
                left_text = buf.get_text(mov_iter, end_iter, True)
            else:
                left_text = ''

            if re.match(r'.*\{\{\s*custom\.$', left_text):
                proposals.append(
                    gtksourceview2.CompletionItem(label='foo', text='foo }}')  # optionally proposed based on left search via regex
                )
                proposals.append(
                    gtksourceview2.CompletionItem(label='bar', text='bar }}')  # optionally proposed based on left search via regex
                )

        context.add_proposals(self, proposals, True)
        return

gobject.type_register(CustomCompletionProvider)


class SimpleProgram(object):

    def __init__(self):
        # self.builder = Gtk.Builder()
        # gobject.type_register(gtksourceview2.View)
        # self.builder.add_from_file("main.glade")
        # self.main_window = self.builder.get_object("MainWindow")
        self.main_window = gtk.Window()
        # self.view = self.builder.get_object("View")
        self.view = gtksourceview2.View()
        self.main_window.add(self.view)
        self.main_window.set_size_request(200, 100)
        self.textbuff = gtksourceview2.Buffer()
        self.view.set_buffer(self.textbuff)
        self.lm = gtksourceview2.LanguageManager()
        self.textbuff.set_language(self.lm.get_language('python'))
        self.main_window.connect("destroy", gtk.main_quit)

        self.keywords = """
                GtkSourceView
                Completion
            """

    def show(self):
        self.set_auto_completation()
        self.main_window.show_all()

    def set_auto_completation(self):
        """
        1)
        Set up a provider that get words from what has already been entered
        in the gtkSource.Buffer that is tied to the GtkSourceView
        2)
        Set up a second buffer that stores the keywords we want to be available
        3)
        Setup an instance of our custome completion class to handle special characters with
        auto complete.
        """
        # This gets the GtkSourceView completion that's already tied to the GtkSourceView
        # We need it to attached our providers to it
        self.view_completion = self.view.get_completion()

        # 1) Make a new provider, attach it to the main buffer add to view_autocomplete
        self.view_autocomplete = gtksourceview2.CompletionWords('main')
        self.view_autocomplete.register(self.textbuff)
        self.view_completion.add_provider(self.view_autocomplete)

        # 2) Make a new buffer, add a str to it, make a provider, add it to the view_autocomplete
        self.keybuff = gtksourceview2.Buffer()
        self.keybuff.begin_not_undoable_action()
        self.keybuff.set_text(self.keywords)
        self.keybuff.end_not_undoable_action()
        self.view_keyword_complete = gtksourceview2.CompletionWords('keyword')

        self.view_keyword_complete.register(self.keybuff)
        self.view_completion.add_provider(self.view_keyword_complete)

        # 3) Set up our custom provider for syntax completion.
        custom_completion_provider = CustomCompletionProvider()
        # custom_completion_provider.do_populate()
        custom_completion_provider.do_get_activation()
        custom_completion_provider.register(self.textbuff)
        print dir(custom_completion_provider)
        item1 = gtksourceview2.CompletionItem(label='Hello World!', text='Hello World!')
        # gtk_proposal = gtksourceview2.CompletionProposal([item1])
        # print custom_completion_provider.activate_proposal(gtk_proposal)
        # self.view.set_buffer(self.text_buff)
        # gtksourceview2.CompletionItem()
        # print custom_completion_provider.register.__doc__
        self.view_completion.add_provider(custom_completion_provider)
        self.custom_completion_provider = custom_completion_provider
        return


def main():
    gui = SimpleProgram()
    gui.show()
    gtk.main()

if __name__ == '__main__':
    main()


# class MyCompletionProvider(gobject.GObject, gtksourceview2.CompletionProvider):
#
#     def __init__(self):
#         gobject.GObject.__init__(self)
#
#     def do_get_name(self):
#         return 'PythonKeywords'
#
#     def do_get_activation(self):
#         return gtksourceview2.COMPLETION_ACTIVATION_USER_REQUESTED
#
#     def do_match(self, context):
#         return True
#
#     def do_get_start_iter(self, context):
#         return context.get_iter()
#
#     def do_activate_proposal(self, proposal, iter):
#         return True
#
#     def do_populate(self, context):
#         self.completions = []
#         for compl in keyword.kwlist:
#             self.completions.append(
#                 gtksourceview2.CompletionItem(
#                     compl.name, compl.complete, info=compl.type))
#         context.add_proposals(self, self.completions, True)
#
#
# gobject.type_register(MyCompletionProvider)
#
#
# class Application(gtk.Window):
#
#     def __init__(self):
#         gtk.Window.__init__(self, gtk.WINDOW_TOPLEVEL)
#         self.connect('destroy', lambda e: gtk.main_quit())
#         self.set_size_request(640, 480)
#         self.__buffer = gtksourceview2.Buffer()
#         self.__buffer.set_text('a' * 20)
#         self.__editor = gtksourceview2.View(self.__buffer)
#         self.__completion_window = self.__editor.get_completion()
#         self.__completion_window.add_provider(MyCompletionProvider())
#         self.add(self.__editor)
#         self.show_all()
#         self.__completion_window.show()
#
# app = Application()
# gtk.main()


# war im SourceEditorController
#
#         import gtksourceview2
#         provider = gtksourceview2.CompletionWords()
#
#         self.textbuff = gtksourceview2.Buffer()
#         self.view.set_buffer(self.textbuff)
#         self.lm = gtksourceview2.LanguageManager()
#         self.textbuff.set_language(self.lm.get_language('python'))
#
#         self.keywords = """
#                 GtkSourceView
#                 Completion
#             """
#
#         # 1) Make a new provider, attach it to the main buffer add to view_autocomplete
#         self.view_completion = self.view.textview.get_completion()
#         self.view_autocomplete = gtksourceview2.CompletionWords.new('main')
#         self.view_autocomplete.register(self.textbuff)
#         comp = gtksourceview2.Completion(select_on_show=True, show_headers=True, show_icons=True, view=self.view.textview)
#         # accelerators, auto-complete-delay, proposal-page-size, provider-page-size, remember-info-visibility,
#         comp.add_provider(provider)
#
#         # 2) Make a new buffer, add a str to it, make a provider, add it to the view_autocomplete
#         self.keybuff = gtksourceview2.Buffer()
#         self.keybuff.begin_not_undoable_action()
#         self.keybuff.set_text(self.keywords)
#         self.keybuff.end_not_undoable_action()
#         self.view_keyword_complete = gtksourceview2.CompletionWords.new('keyword')
#         self.view_keyword_complete.register(self.keybuff)
#         self.view_completion.add_provider(self.view_keyword_complete)
#
#         # # 3) Set up our custom provider for syntax completion.
#         # custom_completion_provider = CustomCompletionProvider()
#         # self.view_completion.add_provider(custom_completion_provider)
#         # self.custom_completion_provider = custom_completion_provider
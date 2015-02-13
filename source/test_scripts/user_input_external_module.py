#!/usr/bin/env python

import gtk
from threading import Condition
import time


class UserInput(object):

    def __init__(self):
        print "User Input initialized"
        self.builder = gtk.Builder()
        self.builder.add_from_file("../../test_scripts/user_input.glade")
        self.builder.connect_signals(self)
        self.builder.get_object("window1").hide()
        self.next_signal = None
        self.next_signal_condition = Condition()
        self.thread_running = True
        self.thread_currently_notifying = False

    def start(self):
        self.builder.get_object("window1").show()
        print "UserInput: started"

    def stop(self):
        self.builder.get_object("window1").hide()
        print "UserInput: stopped"

    def pause(self):
        #how to pause?
        print "UserInput: paused"

    # def get_widget(self, name):
    #     return self.builder.get_object(name)

    def on_window1_destroy(self, *args):
        print "window1 destroyed"
        self.stop()

    def notify_next_signal(self):
        try:
             self.next_signal_condition.acquire()
             self.next_signal_condition.notify_all()
        finally:
             self.next_signal_condition.release()

    def get_next_signal(self, timeout):
        try:
            self.next_signal_condition.acquire()
            self.next_signal_condition.wait(timeout)
        finally:
            self.next_signal_condition.release()
        #very ugly sleep; without sleep the caller for get_next_signal might return too fast,
        # which causes the python condition to crash
        time.sleep(0.05)
        return_signal = self.next_signal
        self.next_signal = None
        return return_signal

    def on_turn_left_button_clicked(self, widget, data=None):
        print "turn_left buttton clicked"
        self.next_signal = "turn_left"
        self.notify_next_signal()

    def on_turn_right_button_clicked(self, widget, data=None):
        print "turn_right buttton clicked"
        self.next_signal = "turn_right"
        self.notify_next_signal()

    def on_forward_button_clicked(self, widget, data=None):
        print "forward buttton clicked"
        self.next_signal = "forward"
        self.notify_next_signal()

    def on_backward_button_clicked(self, widget, data=None):
        print "backward buttton clicked"
        self.next_signal = "backward"
        self.notify_next_signal()
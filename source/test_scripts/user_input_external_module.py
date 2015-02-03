#!/usr/bin/env python
#-*- coding: utf-8 -*-

import gtk
from threading import Thread, Condition
import ctypes
import gobject
import time
import sys


# gtk.gdk.threads_init()

def terminate_thread(thread):
    """Terminates a python thread from another thread.

    :param thread: a threading.Thread instance
    """
    if not thread.isAlive():
        return

    exc = ctypes.py_object(SystemExit)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("nonexistent thread id")
    elif res > 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


class GTKWorkerThread(Thread, object):

    def __init__(self):
        Thread.__init__(self)
        self.builder = gtk.Builder()
        #self.builder.add_from_file("user_input.glade")
        self.builder.add_from_file("../../test_scripts/user_input.glade")
        self.builder.connect_signals(self)
        self.next_signal = None
        self.next_signal_condition = Condition()
        self.thread_running = True

    def check_exit_flag(self):
        if not self.thread_running:
            print "window1 destroyed"
            self.builder.get_object("window1").hide()
            gtk.main_quit()
            print "after gtk.main_quit"
            return
        gobject.timeout_add(100, self.check_exit_flag)

    def stop(self):
        self.thread_running = False

    def get_widget(self, name):
        return self.builder.get_object(name)

    def run(self):
        gobject.timeout_add(100, self.check_exit_flag)
        self.thread_running = True
        print "UserInput: started"
        gtk.main()

    def on_window1_destroy(self, *args):
        #print "window1 destroyed"
        self.thread_running = False
        #print "Call main quit!"
        #gtk.main_quit()

    def notify_next_signal(self):
        self.next_signal_condition.acquire()
        self.next_signal_condition.notify_all()
        self.next_signal_condition.release()

    def get_next_signal(self):
        try:
            self.next_signal_condition.acquire()
            self.next_signal_condition.wait()
        finally:
            self.next_signal_condition.release()
        return self.next_signal

    def on_turn_left_button_clicked(self, widget, data=None):
        print "on turn left buttton clicked"
        self.next_signal = "turn_left"
        self.notify_next_signal()

    def on_turn_right_button_clicked(self, widget, data=None):
        self.next_signal = "turn_right"
        self.notify_next_signal()

    # def right_button_clicked_cb(self, widget, data=None):
    #     self.next_signal = "right"
    #     self.notify_next_signal()
    #
    # def on_left_button_clicked(self, widget, data=None):
    #     self.next_signal = "left"
    #     self.notify_next_signal()

    def on_forward_button_clicked(self, widget, data=None):
        self.next_signal = "forward"
        self.notify_next_signal()

    def on_backward_button_clicked(self, widget, data=None):
        self.next_signal = "backward"
        self.notify_next_signal()


class UserInput(object):

    def __init__(self):

        print "---------------------------------- User Input initialized"
        self.thread_running = False
        self.gtk_worker_thread = None

    def start(self):
        self.gtk_worker_thread = GTKWorkerThread()
        self.gtk_worker_thread.start()

    def stop(self):
        # w = self.gtk_worker_thread.get_widget("window1")
        # if not w.emit("delete-event", gtk.gdk.Event(gtk.gdk.DELETE)):
        #      w.destroy()
        try:
            self.gtk_worker_thread.stop()
        except RuntimeError:
            pass
        print "UserInput: stopped"

    def get_next_signal(self):
        return self.gtk_worker_thread.get_next_signal()

    def pause(self):
        #how to pause?
        print "UserInput: paused"
import gtk
from gtkmvc import Controller
from pylint import epylint as lint

from utils import log
logger = log.get_logger(__name__)
import statemachine.singleton


#TODO: comment

class SourceEditorController(Controller):
    # TODO Missing functions
    # - Code function-expander
    # - Code completion

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

    def register_view(self, view):
        view.get_buffer().connect('changed', self.code_changed)
        view['apply_button'].connect('clicked', self.apply_clicked)
        view['cancel_button'].connect('clicked', self.cancel_clicked)
        view.set_text(self.model.state.script.script)

    def register_adapters(self):
        pass

    #===============================================================
    def code_changed(self, source):
        #print "The text in the text_buffer changed"
        self.view.apply_tag('default')

    #===============================================================
    def apply_clicked(self, button):
        logger.debug("Apply button pressed!")
        tbuffer = self.view.get_buffer()
        current_text = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())
        text_file = open("/tmp/file_to_get_pylinted.py", "w")
        text_file.write(current_text)
        text_file.close()

        (pylint_stdout, pylint_stderr) = lint.py_run('/tmp/file_to_get_pylinted.py', True)
        pylint_stdout_data = pylint_stdout.readlines()
        pylint_stderr_data = pylint_stderr.readlines()

        logger.debug("pylint_stdout_data: %s" % pylint_stdout_data)
        logger.debug("pylint_stderr: %s" % pylint_stderr_data)

        invalid_sytax = False
        for elem in pylint_stdout_data:
            if " Error " in elem or " error " in elem:
                invalid_sytax = True

        if invalid_sytax:
            message = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
            message_string = "Are you sure you want the save this file \nThe following errors were found:"
            for elem in pylint_stdout_data:
                if " Error " in elem or " error " in elem:
                    message_string = "%s \n %s " % (message_string, str(elem))
            message.set_markup(message_string)
            message.add_button("Yes", 42)
            message.add_button("No", 43)
            message.connect('response', self.on_message_dialog_response_signal, current_text)
            message.show()
        else:
            if self.model.state.set_script_text(current_text):
                logger.debug("File saved")
                statemachine.singleton.global_storage.save_script_file(self.model.state)
            self.view.set_text(self.model.state.script.script)

    #===============================================================
    def cancel_clicked(self, button):
        self.view.set_text(self.model.state.script.script)

    def on_message_dialog_response_signal(self, widget, response_id, current_text):
        if response_id == 42:
            self.model.state.script.script = current_text
            self.view.set_text(self.model.state.script.script)
            logger.debug("File saved")
        else:
            logger.debug("File not saved")
        widget.destroy()

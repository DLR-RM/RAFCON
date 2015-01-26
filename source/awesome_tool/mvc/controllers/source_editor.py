import gtk
from gtkmvc import Controller
from pylint import epylint as lint

from utils import log
logger = log.get_logger(__name__)


class SourceEditorController(Controller):
    """Controller handling the source script of a state

    :param mvc.models.StateModel model: The state model containing the data
    :param mvc.views.SourceEditorView view: The GTK view showing the data as a table
    """

    # TODO Missing functions
    # - Code function-expander
    # - Code completion

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view.get_buffer().connect('changed', self.code_changed)
        view['apply_button'].connect('clicked', self.apply_clicked)
        view['cancel_button'].connect('clicked', self.cancel_clicked)
        view.set_text(self.model.state.script.script)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    #===============================================================
    def code_changed(self, source):
        #print "The text in the text_buffer changed"
        self.view.apply_tag('default')

    #===============================================================
    def apply_clicked(self, button):
        print "Apply button pressed!"
        tbuffer = self.view.get_buffer()
        current_text = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())
        text_file = open("/tmp/file_to_get_pylinted.py", "w")
        text_file.write(current_text)
        text_file.close()

        (pylint_stdout, pylint_stderr) = lint.py_run('/tmp/file_to_get_pylinted.py', True)
        pylint_stdout_data=pylint_stdout.readlines()
        pylint_stderr_data=pylint_stdout.readlines()

        print "pylint_stdout_data: ", pylint_stdout_data
        print "pylint_stderr: ", pylint_stderr

        invalid_sytax = False
        for elem in pylint_stdout_data:
            if " Error " in elem or " error " in elem:
                #print elem
                invalid_sytax = True

        if invalid_sytax:
            #print "There are still errors in the python file"
            message = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
            message_string = "Are you sure you want the save this file \nThe following errors were found:"
            for elem in pylint_stdout_data:
                if " Error " in elem or " error " in elem:
                    message_string = "%s \n %s " % (message_string, str(elem))
                #print message_string
            message.set_markup(message_string)
            message.add_button("Yes", 42)
            message.add_button("No", 43)
            message.connect('response', self.on_message_dialog_response_signal, current_text)
            message.show()
        else:
            self.model.state.script.script = current_text
            self.view.set_text(self.model.state.script.script)
            logger.debug("File saved")

    #===============================================================
    def cancel_clicked(self, button):
        self.view.set_text(self.model.state.script.script)

    def on_message_dialog_response_signal(self, widget, response_id, current_text):
        #print current_text
        if response_id == 42:
            self.model.state.script.script = current_text
            self.view.set_text(self.model.state.script.script)
            logger.debug("File saved")
        else:
            logger.debug("File not saved")
        widget.destroy()

if __name__ == '__main__':
    from mvc.views import SourceEditorView, SingleWidgetWindowView
    from mvc.controllers import SingleWidgetWindowController

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()

    v = SingleWidgetWindowView(SourceEditorView, width=550, height=500, title='Source Editor')
    c = SingleWidgetWindowController(ctr_model, v, SourceEditorController)

    gtk.main()

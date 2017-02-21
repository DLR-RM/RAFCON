import gtk
import threading
import testing_utils

from rafcon.gui.utils import dialog
from rafcon.utils import log

logger = log.get_logger(__name__)


def xor(_list):
    result = 0
    for element in _list:
        result = element.get_active() ^ result
    return result


def trigger_dialog_tests():
    test_text = "test_text"

    def on_ok_clicked(widget, response_id):
        # Default response by the "ok" button is -5
        assert response_id == -5

    def on_button_clicked(widget, response_id):
        # Last button in the order gets the reponse 4 ((index=3) + 1)
        assert response_id == 4

    def on_entry_activated(widget, response_id):
        # The entry gets the same response as the first button
        assert response_id == 1
        assert widget.entry.get_text() == test_text
        assert widget.check.get_label() == test_text
        # As gtk.CheckButton is a ToggleButton, get_active()=True represents the "checked" state
        assert widget.check.get_active()

    def on_checkbox_dialog_approval(widget, response_id):
        # Check if all checkboxes appeared and the modulo operation worked
        assert not widget.get_checkbox_state_by_index(0)
        assert widget.get_checkbox_state_by_index(3)
        # Check if the checkbutton labels are correct
        assert len(widget.get_checkbox_state_by_name(test_text)) == 4

        assert not xor(widget.get_checkboxes())

    dialog_window = dialog.RAFCONMessageDialog(markup_text=test_text, callback=on_ok_clicked)
    # First button of the dialog, in this case the standard "Ok" button
    button = dialog_window.get_action_area().get_children()[0]
    button.clicked()
    dialog_window.destroy()
    button_texts = ["First", "Second", "Third", "Fourth"]
    dialog_window = dialog.RAFCONButtonDialog(markup_text=test_text,
                                              callback=on_button_clicked,
                                              button_texts=button_texts)

    for index, button_text in enumerate(button_texts):
        # Check if the button order is the same as requested by the button_texts list
        button = dialog_window.hbox.get_children()[index]
        assert str(button.get_label()) == button_texts[index]

    button.clicked()
    dialog_window.destroy()

    dialog_window = dialog.RAFCONInputDialog(markup_text=test_text,
                                             callback=on_entry_activated,
                                             checkbox_text=test_text,
                                             button_texts=button_texts)
    dialog_window.entry.set_text(test_text)
    dialog_window.check.set_active(True)
    # Represents hitting the enter button
    dialog_window.entry.activate()
    dialog_window.destroy()

    dialog_window = dialog.RAFCONColumnCheckBoxDialog(markup_text=test_text,
                                                      callback=on_checkbox_dialog_approval,
                                                      message_type=gtk.MESSAGE_QUESTION,
                                                      checkbox_texts=[test_text, test_text, test_text, test_text],
                                                      button_texts=button_texts)

    for index, checkbox in enumerate(dialog_window.checkboxes):
        # Check the checkboxes in an alternating order
        checkbox.set_active(index % 2)

    button = dialog_window.get_action_area().get_children()[0]
    button.clicked()

    dialog_window.destroy()


def test_dialog_test(caplog):

    testing_utils.initialize_environment()

    logger.debug("Dialog test started.")

    thread = threading.Thread(target=trigger_dialog_tests)
    thread.start()
    testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0)


if __name__ == '__main__':
    test_dialog_test(None)
    # pytest.main([__file__, '-xs'])

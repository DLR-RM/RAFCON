import gtk
from gtkmvc import View


class SingleWidgetWindowView(View):

    def __init__(self, view_class, width=500, height=500, title=None):
        View.__init__(self)

        w = gtk.Window()
        if title is None:
            w.set_title(str(view_class))
        else:
            w.set_title(title)
        w.resize(width=width, height=height)
        self.widget_view = view_class()
        w.add(self.widget_view.get_top_widget())
        w.show_all()

        self['main_frame'] = self.widget_view
        self['main_window'] = w

    pass  # class end


if __name__ == '__main__':
    from mvc.views.source_editor import SourceEditorView
    from mvc.controllers import SourceEditorController, SingleWidgetWindowController

    from statemachine.states.execution_state import ExecutionState as State
    from mvc.models import StateModel, ContainerStateModel
    state1 = State('Rico2')
    m = StateModel(state1)

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state] = main.main()

    v = SingleWidgetWindowView(SourceEditorView, title='Source Editor')
    c = SingleWidgetWindowController(ctr_model, v, SourceEditorController)

    gtk.main()
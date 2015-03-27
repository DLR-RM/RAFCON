class History:

    def __init__(self, state_machine_model):
        self.state_machine_model = state_machine_model

    def meta_changed_notify_after(self, changed_parent_model, changed_model, recursive_changes):
        """
        :param changed_parent_model awesome_tool.mvc.models.container_state.ContainerStateModel: model that holds the changed model
        :param changed_model gtkmvc.Model: inherent class of gtkmvc.Model like TransitionModel, StateModel and so on
        :param recursive_changes bool: indicates if the changes are recursive and touch multiple or all recursive childs
        :return:
        """
        # store meta data

        # -> in case of undo/redo overwrite Model.meta-dict

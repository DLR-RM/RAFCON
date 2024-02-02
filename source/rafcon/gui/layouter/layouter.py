from gaphas.solver import Variable
from rafcon.core.states.container_state import ContainerState
from rafcon.gui.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateMachineLayouter:
    """
    StateMachineLayouter gets a state machine, and layouts it in a particular way.
    """

    def __init__(self, root_state):
        self.root_state = root_state

        self.x_gap = 25
        self.y_gap = 50
        self.state_width = 80.
        self.state_height = 80.

        self.scale_factor = 1
        self.previous_container_state = None

        # positions where an income or an outcome can occur
        self.up_pos = (self.state_width / 2., 0.)
        self.down_pos = (self.state_width / 2., self.state_height)
        self.left_pos = (0., self.state_height / 4.)
        self.right_pos = (self.state_width, self.state_height / 4.)

    def layout_state_machine(self):
        """This function will format the state machine in a merlon like format."""
        if not self.root_state.states:
            return
        self.__layout(self.root_state)

    @staticmethod
    def __sort_states_in_container_state(container_state):
        """Sort states in container state depending on state transitions."""
        if not container_state.states.keys():
            return []
        if len(container_state.states.keys()) == 1:
            return list(container_state.states.keys())

        states_with_transitions = set()
        for transition in container_state.transitions:
            states_with_transitions.add(transition.transition.from_state)
            states_with_transitions.add(transition.transition.to_state)

        from_state = container_state.core_element.start_state_id
        if not from_state:
            for state in states_with_transitions:
                if state in container_state.states.keys():
                    from_state = state
                    break

        sorted_states = list(container_state.states.keys() - states_with_transitions)
        sorted_states.append(from_state)
        # remove None value from sorted states
        try:
            sorted_states.remove(None)
        except ValueError:
            pass

        add_to_sorted = True
        while len(sorted_states) != len(container_state.states.keys()):

            if not add_to_sorted:
                for state in container_state.states.keys():
                    if state not in sorted_states:
                        from_state = state
                        sorted_states.append(from_state)

            for transition in container_state.transitions:
                if transition.transition.from_state == from_state \
                        and transition.transition.to_state in container_state.states.keys() \
                        and transition.transition.to_state not in sorted_states:
                    from_state = transition.transition.to_state
                    sorted_states.append(from_state)
                    add_to_sorted = True
                    break
                else:
                    add_to_sorted = False

        return sorted_states

    @staticmethod
    def __get_target_state_dimensions(canvas_width, canvas_height):
        """
        get_target_state_dimensions receives a desired canvas width and height, and returns the overall rootstate size,
        and the border width.

        :param canvas_width: The width of the canvas.
        :param canvas_height: The height of the canvas.
        :return: (double, double, double): (width, height, border_width).
        """
        border_width = Variable(min(canvas_width, canvas_height) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR)
        r_width = canvas_width + 2 * border_width
        r_height = canvas_height + 2 * border_width
        border_width = Variable(min(r_width, r_height) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR)
        while (r_width - 2 * border_width) < canvas_width and (r_height - 2 * border_width) < canvas_height:
            r_width = r_width + 2 * border_width
            r_height = r_height + 2 * border_width
            border_width = Variable(min(r_width, r_height) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR)
        return r_width, r_height, border_width

    def __layout(self, container_state):
        """Layout container state.

        Position states one after another in one column
        and resize container state and child states.
        """
        sorted_states = self.__sort_states_in_container_state(container_state)

        column = 0
        previous_state = None
        states = []
        for state in sorted_states:
            state_m = container_state.states.get(state)
            if state_m:
                self.__position_state(state_m, previous_state, column)
                column += 1
                previous_state = state_m
                states.append(state)

        self.__resize_container_state(container_state, states)

    def __position_state(self, state, previous_state, column):
        """Puts state in right position, resize it and set income and outcome positions."""
        income_pos = self.left_pos
        outcome_pos = self.right_pos

        # set state size
        state.set_meta_data_editor(
            'size',
            (self.state_width, self.state_height)
        )
        state.set_meta_data_editor(
            'name.size',
            (self.state_width / 0.9, 20)
        )

        # set position of income and outcome
        state.income.set_meta_data_editor('rel_pos', income_pos)
        outcomes = [oc for oc in state.outcomes if oc.outcome.outcome_id >= 0]
        if outcomes:
            out_come = [oc for oc in state.outcomes if oc.outcome.outcome_id >= 0].pop()
            out_come.set_meta_data_editor(
                'rel_pos',
                outcome_pos
            )

        # set position of state
        if previous_state:
            width = previous_state.meta['gui']['editor_gaphas']['size'][0]
            previous_state_x = previous_state.meta['gui']['editor_gaphas']['rel_pos'][0]
            current_x = previous_state_x + width + self.x_gap + 10
        else:
            width = self.state_width
            current_x = column * (self.x_gap + width) + self.x_gap + 10
        current_y = self.y_gap

        state.set_meta_data_editor(
            'rel_pos',
            (current_x, current_y)
        )

        if isinstance(state.state, ContainerState):
            self.__layout(state)

    def __resize_container_state(self, container_state, states):
        """Resizing container state depending on the position and size of the first and the last states."""
        # if only one state inside container state,
        # we need to resize container state depending only on size of the first state
        if len(states) == 0:
            canvas_width = self.state_width
        elif len(states) == 1:
            first_state = container_state.states[states[0]]
            canvas_width = first_state.meta['gui']['editor_gaphas']['size'][0] + self.x_gap
        else:
            first_state = container_state.states[states[0]]
            last_state = container_state.states[states[-1]]
            last_state_width = last_state.meta['gui']['editor_gaphas']['size'][0]
            first_state_x = first_state.meta['gui']['editor_gaphas']['rel_pos'][0]
            last_state_x = last_state.meta['gui']['editor_gaphas']['rel_pos'][0]

            canvas_width = last_state_x - first_state_x + last_state_width + self.x_gap

        if self.previous_container_state and container_state.parent != self.previous_container_state.parent:
            self.scale_factor += 1
        self.previous_container_state = container_state
        label_height = 10
        canvas_height = (self.state_height * self.scale_factor) + (self.y_gap * self.scale_factor)

        # root state width, height, and root state border size.
        r_width, r_height, border_size = self.__get_target_state_dimensions(
            canvas_width,
            canvas_height,
        )

        # set root state size
        container_state.set_meta_data_editor(
            'size',
            (r_width, r_height)
        )

        # set root state in / outcome position
        container_state.income.set_meta_data_editor(
            'rel_pos',
            (0., label_height + border_size + self.y_gap + self.state_height / 4.)
        )
        out_comes = [oc for oc in container_state.outcomes if oc.outcome.outcome_id == 0]
        if out_comes:
            out_come = out_comes.pop()
            out_come.set_meta_data_editor(
                'rel_pos',
                (r_width, label_height + border_size + self.y_gap + self.state_height / 4.)
            )

        # last state is a special case, its outcome should always be right.
        if not states:
            return
        out_comes = [
            oc
            for oc in container_state.states[states[-1]].outcomes
            if oc.outcome.outcome_id == 0
        ]
        if out_comes:
            out_come = out_comes.pop()
            out_come.set_meta_data_editor(
                'rel_pos',
                self.right_pos
            )

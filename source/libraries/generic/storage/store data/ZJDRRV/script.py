

def execute(self, inputs, outputs, gvm):
    from awesome_tool.mvc.statemachine_helper import StateMachineHelper
    
    # self.parent refers to the Library state, but we want its parent
    state = self.parent.parent
    state_m = StateMachineHelper.get_state_model_for_state(state)
    
    # Store the data in the meta data of the next higher root state
    # (either a library or the root state of the state machine)
    root_state_m = StateMachineHelper.get_root_state_model(state_m, library_root=True)
    root_state_m.meta['data'][inputs['key']] = inputs['value']
    
    # Set the mark dirty flag
    # This tells the state machine that is was changed
    # It causes an asterix symbol to be shown, the data is then saved with the next save
    state_machine_m = StateMachineHelper.get_state_machine_model_for_state(state)
    state_machine_m.marked_dirty = True
    return 0


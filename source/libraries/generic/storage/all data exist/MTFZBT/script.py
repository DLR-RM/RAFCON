

def execute(self, inputs, outputs, gvm):
    from awesome_tool.mvc.statemachine_helper import StateMachineHelper
    
    # self.parent refers to the Library state, but we want its parent
    state = self.parent.parent
    state_m = StateMachineHelper.get_state_model_for_state(state)
    
    # Load the data from the meta data of the next higher root state
    # (either a library or the root state of the state machine)
    root_state_m = StateMachineHelper.get_root_state_model(state_m, library_root=True)
    
    for key in inputs['keys']:
        if not isinstance(key, (str, basestring)):
            self.logger.error("Keys must be of type str")
            return "aborted"
    
        if key not in root_state_m.meta['data']:
            return "no"
    
    return "yes"



def execute(self, inputs, outputs, gvm):
    with open(str(gvm.get_variable('tmp_path')), 'a') as f:
        f.write("'" + self.state_id + " wrote to file!'\n")
    return 0

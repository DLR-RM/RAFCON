
def execute(self, inputs, outputs, gvm):
    with open('/tmp/test_file', 'a') as f:
        f.write("'" + self.state_id + " wrote to file!'\n")
    return 0

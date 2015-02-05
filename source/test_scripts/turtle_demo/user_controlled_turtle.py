import copy

def execute(self, inputs, outputs, external_modules, gvm):
    print "User controlled turtle state is executing ..."
    print "Waiting for user input"
    user_input = copy.copy(external_modules["user_input"].instance.get_next_signal(3))
    print user_input
    if user_input == "forward":
        external_modules["ros"].instance.move_turtle("turtle1", 2, 0, 0)
    if user_input == "backward":
        external_modules["ros"].instance.move_turtle("turtle1", -2, 0, 0)
    if user_input == "turn_left":
        external_modules["ros"].instance.move_turtle("turtle1", 0, 0, 1)
    if user_input == "turn_right":
        external_modules["ros"].instance.move_turtle("turtle1", 0, 0, -1)
    return 0

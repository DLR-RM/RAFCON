def execute(self, inputs, outputs, external_modules, gvm):
    print "Create new turtle ..."
    external_modules["ros"].instance.spawn_turtle("bot_turtle", 1, 2, 1)
    return 0

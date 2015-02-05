def execute(self, inputs, outputs, external_modules, gvm):
    print "Create new turtle ..."
    # 4 quadrant
    #external_modules["ros"].instance.spawn_turtle("bot_turtle", 1, 2, 1)
    # 3 quadrant
    external_modules["ros"].instance.spawn_turtle("bot_turtle", 9, 2, 1)
    return 0

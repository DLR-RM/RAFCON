class TestModule:

    def __init__(self):
        self.my_first_var = 10

    def start(self):
        print "External module: started"
        print "External module: my_first_var is " + str(self.my_first_var)

    def stop(self):
        print "External module: stopped"

    def pause(self):
        print "External module: paused"

    def custom_function(self):
        print "External module: this is a custom function"
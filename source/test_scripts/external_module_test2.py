class TestModule2:

    def __init__(self):
        print "External module: init of TestModule2 called"
        self.my_first_var = 10

    def start(self):
        print "External module2: started"
        print "External module2: my_first_var is " + str(self.my_first_var)

    def stop(self):
        print "External module2: stopped"

    def pause(self):
        print "External module2: paused"

    def custom_function(self):
        '''
            This is a test docu that can be retrieved via: import inspect, inspect.getmembers() and inspect.getdoc()
        '''
        print "External module2: this is a custom function"

    def custom_function2(self):

        print "External module2: this is a custom function 2"

    def custom_function3(self):
        print "External module2: this is a custom function 3"
class TestModule2:

    def __init__(self):
        '''
            Dummy Comment
        '''
        print "External module: init of TestModule2 called"
        self.my_first_var = 10

    def start(self):
        '''
            Dummy Comment
        '''
        print "External module2: started"
        print "External module2: my_first_var is " + str(self.my_first_var)

    def stop(self):
        '''
            Dummy Comment
        '''
        print "External module2: stopped"

    def pause(self):
        '''
            Dummy Comment
        '''
        print "External module2: paused"

    def custom_function(self):
        '''
            Dummy Comment
        '''
        print "External module2: this is a custom function"

    def custom_function2(self):
        '''
            Dummy Comment
        '''
        print "External module2: this is a custom function 2"

    def custom_function3(self):
        '''
            Dummy Comment
        '''
        print "External module2: this is a custom function 3"
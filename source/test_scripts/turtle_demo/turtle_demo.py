import random
import math
def enter(self, scoped_variables, external_modules, gvm):
    print "Enter turtle demo preemptive concurrency state"
    external_modules["ros"].instance.spawn_turtle("food0", 1, 1, random.uniform(0, 2*math.pi))
    external_modules["ros"].instance.spawn_turtle("food1", 10, 1, random.uniform(0, 2*math.pi))
    external_modules["ros"].instance.spawn_turtle("food2", 1, 10, random.uniform(0, 2*math.pi))
    external_modules["ros"].instance.spawn_turtle("food3", 10, 10, random.uniform(0, 2*math.pi))

def exit(self, scoped_variables, external_modules, gvm):
    print "Exit turtle demo preemptive concurrency state"

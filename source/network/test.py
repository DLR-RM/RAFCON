import time
from twisted.internet import reactor

def shutdown():
    time.sleep(1)
    print "stopping"
    reactor.callFromThread(reactor.stop)

print "hello world"
reactor.callInThread(shutdown)
print "run reactor"
reactor.run()

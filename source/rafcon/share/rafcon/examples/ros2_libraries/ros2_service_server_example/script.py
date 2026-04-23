# RAFCON ROS2 Service Server State Machine
# Author: Johannes Ernst
#
# NOTE: To use the state machine, ROS2 needs to be installed (tested with 'Humble')

from rafcon.utils.ros2.service_server import ServiceServerHandle
from rafcon.utils import log

LOGGER = log.get_logger('service_callback')
RESPONSE = None

def srv_callback(req, resp):
    # This is a function to handle the service request from 
    # the server side. The callback is triggered once a client
    # request arrives. This has to be implemented:
    global RESPONSE
    LOGGER.info(f"Incoming request: {req}")
    resp.sum = req.a + req.b
    RESPONSE = resp
    return resp


def execute(self, inputs, outputs, gvm):
    # Define service definition and name
    from example_interfaces.srv import AddTwoInts
    srv_definition = AddTwoInts
    srv_name = inputs['namespace'] + 'add_two_ints'

    # Get node and create server
    node = gvm.get_variable('rafcon_ros_node')
    server = ServiceServerHandle(self, 
                                 node,
                                 srv_definition, 
                                 srv_name, 
                                 srv_callback)

    # Wait for a service request or until preempted
    global RESPONSE
    result =  "success"
    while RESPONSE is None:
        self.logger.info("Waiting for service request...")
        if self.wait_for_interruption(1):
            if self.preempted:
                result = "preempted"
                break
            if self.paused:
                self.wait_for_unpause(0.1)
                if self.preempted:
                    result = "preempted"
                    break

    # Destroy service and return
    node.destroy_service(server)
    return result
# RAFCON ROS2 Service Call State Machine
# Author: Johannes Ernst
#
# NOTE: To use the state machine, ROS2 needs to be installed (tested with 'Humble')

import rclpy

from rafcon.utils.ros2.service_client import ServiceClientHandle


def set_req(cli, inputs):
    # Map the RAFCON inputs to service request (defined in .srv definition).
    # This implementation maps rafcon inputs to service request 1-on-1.
    # If the field keys do not match, please adapt this function
    # using e.g. cli.set_srv_req_field_from_inputs or remove this function and
    # manually set the srv fields in the code below (see example).
    for key, value in inputs.items():
        if hasattr(cli.srv_req, key):
            cli.set_srv_req_field_from_inputs(key, inputs)

def get_resp(cli, outputs):
    # Map service response (defined in .srv definition) to RAFCON outputs.
    # This implementation maps service response to rafcon outputs 1-on-1
    # If the field keys do not match, please adapt this function
    # using e.g. cli.get_srv_resp_field or remove this function and
    # manually set the srv response fields in the code below (see example).
    for key, value in outputs.items():
        if hasattr(cli.srv_resp, key):
            outputs[key] = cli.get_srv_resp_field(key)


def execute(self, inputs, outputs, gvm):
    # Define service definition and name
    from example_interfaces.srv import AddTwoInts
    srv_definition = AddTwoInts
    srv_name = inputs['namespace'] + '/add_two_ints'
    callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

    # Create service client handle
    client = ServiceClientHandle(self,
                                 gvm,
                                 srv_definition,
                                 srv_name,
                                 callback_group)

    # Execute service call with a lock for thread safety
    with client.lock:

        # Wait for service to become available
        available = client.wait_for_srv()
        if available != "success":
            return available

        # Auto-assemble service request from inputs
        set_req(client, inputs)
        
        # Alternative for manual setting:
        # cli.set_srv_req_field('a', inputs['a'])
        # cli.set_srv_req_field('b', inputs['b'])

        # Call service
        client.call_srv()
        result = client.wait_until_srv_completion()

    # Auto-assign response to outputs
    if result == "success":
        get_resp(client, outputs)
        
        # Alternative for manual getting:
        # outputs['sum'] = cli.get_srv_resp_field('sum')

    return result

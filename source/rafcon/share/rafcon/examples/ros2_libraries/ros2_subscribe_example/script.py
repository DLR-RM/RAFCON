# RAFCON ROS2 Subscribing State Machine
# Author: Johannes Ernst
#
# NOTE: To use the state machine, ROS2 needs to be installed (tested with 'Humble')

from rafcon.utils.ros2.subscriber import SubscriberHandle


def get_msg(sub, outputs):
    # Map received message (defined in .msg definition) to RAFCON outputs.
    # This implementation maps received response to rafcon outputs 1-on-1.
    # If the field keys do not match, please adapt this function 
    # using e.g. sub.get_sub_msg_field or remove this function and
    # manually get the msg fields in the code below (see example).
    for key, value in outputs.items():
        if hasattr(sub.sub_msg, key):
            outputs[key] = sub.get_sub_msg_field(key)


def execute(self, inputs, outputs, gvm):
    # Define message definition and topic
    from std_msgs.msg import String
    msg_definition = String
    topic = inputs['namespace'] + '/chatter'
    sub_msg_timeout = inputs['timeout']

    # Get node and create subscriber
    node = gvm.get_variable('rafcon_ros_node')
    sub = SubscriberHandle(self, node, msg_definition, topic)

    # Wait for the subscription and grab message
    result = sub.wait_for_sub_msg(timeout=sub_msg_timeout)

    # Auto-assign message to outputs
    get_msg(sub, outputs)
    
    # Alternative for manual getting:
    # outputs['data'] = sub.get_sub_msg_field('data')

    return result

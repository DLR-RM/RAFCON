# RAFCON ROS2 Publishing State Machine
# Author: Johannes Ernst
#
# NOTE: To use the state machine, ROS2 needs to be installed (tested with 'Humble')

import rclpy

from rafcon.utils.ros2.publisher import PublisherHandle


def set_msg(pub, inputs):
    # Map the RAFCON inputs to publishing message (defined in .msg definition).
    # This implementation maps rafcon inputs to message fields 1-on-1.
    # If the field keys do not match, please adapt this function
    # using e.g. pub.set_message_field_from_inputs or remove this function and
    # manually set the msg fields in the code below (see example).
    for key, value in inputs.items():
        if hasattr(pub.pub_msg, key):
            pub.set_pub_msg_field_from_inputs(key, inputs)


def execute(self, inputs, outputs, gvm):
    # Define message definition and topic
    from std_msgs.msg import String
    msg_definition = String
    topic = inputs['namespace'] + 'chatter'
    callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

    # Create publisher handle
    pub = PublisherHandle(self,
                          gvm,
                          msg_definition,
                          topic,
                          callback_group=callback_group)

    # Execute publishing with a lock for thread safety
    with pub.lock:
    
        # Auto-assemble message from inputs
        set_msg(pub, inputs)
        
        # Alternative for manual setting:
        # pub.set_pub_msg_field("data", inputs["data"])

        # Wait for a subscriber to be available
        pub.wait_for_sub(timeout=1)

        # Publish message
        return pub.publish_msg()
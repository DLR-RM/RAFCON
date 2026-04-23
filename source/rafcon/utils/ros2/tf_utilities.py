# Copyright (C) 2016-2026 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Johannes Ernst
# Ryo Sakagami
#
# NOTE: To use the scripts, ROS2 needs to be installed (tested with 'Humble')

'''
This file only holds utility functions for ros2 and ros2_tf.
'''


def tf_from_stamped(tf_stamped):
    # To extract translation and quaternion rotation from tf_stamped
    trans = [
        tf_stamped.transform.translation.x,
        tf_stamped.transform.translation.y,
        tf_stamped.transform.translation.z,
    ]
    quat_rot = [
        tf_stamped.transform.rotation.x,
        tf_stamped.transform.rotation.y,
        tf_stamped.transform.rotation.z,
        tf_stamped.transform.rotation.w,
    ]
    return trans, quat_rot

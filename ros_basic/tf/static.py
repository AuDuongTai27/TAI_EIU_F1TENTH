#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped  # the message type that is accepted in the TF and Tf static topic called "TransformStamped"


class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics") 

        # create an instance of the static transform broadcaster which takes as input the instance of the current class
        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)

        # create a new message of this type, # inforrmation: Two frames that are connected by this transform 

        self.static_transform_stamped_ = TransformStamped()

        """
        the first thing: is to add the information about the time when this transform has been generated 
        self.get_clock().now() have ingerited from the node class, to_msg(): convert to a messgae
        """
        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        """
        Let's also define the names of the two frames that are connected by these static transform 
        This static transform that we are going to publish will connect together these two frames here we have just defined 
        """
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"

        """
        define the characteristics of this conenction
        we have to define the roation matrix and the translation vector that defines the connection so now these two frames are connected 
        """

        """
        Define a translation vector  => Basically we are making the bumperbot top frame be on the top of the bumperbot base frame o f 30 cm
        """
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3

        """
        In ros2 and in general in computer science, instead of using rotation matrices and so Euler angles are used, the quaternions the representation of the orientation using
        Quaternions as several advantages compared to the rotation matrices => that we will investigate later in the course
        For the scope of this lesson, we just need to know that a quaternion is composed of 4 components 
        """
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0

        """
        The transform stamped message is completed. So with this simple transform between two frames and now we can use this object here ti send the transform
        """

        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)

        self.get_logger().info("Publishing static transform between %s and %s " % (self.static_transform_stamped_.header.frame_id, self.static_transform_stamped_.child_frame_id))

        """
        Above we need for the simple TF kinematic class to make it publish a static transform between 
        """

def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()




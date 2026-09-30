import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import numpy as np
import logging_mp
logger_mp = logging_mp.getLogger(__name__)

from scipy.spatial.transform import Rotation as R

initial_x = 0.207
initial_y = 0.129
initial_z = 0.067

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'pelvis',
                'left_wrist_yaw_link',
                rclpy.time.Time())
            quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            rotation = R.from_quat(quat)
            rotation_matrix = rotation.as_matrix()


        except TransformException as ex:
            self.get_logger().info(
                f'Could not find transform: {ex}')
            return

        print(rotation_matrix)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
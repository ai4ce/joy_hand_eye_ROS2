import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')  # type: ignore

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)
    
    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=2))
            return transform
        except Exception as e:
            self.get_logger().error('Failed to get transform: %s' % e)
            

def main():
    rclpy.init()
    node = FrameListener()
    transform = node.get_transform('link_base', 'link_eef')
    if transform:
        node.get_logger().info('Transform: %s' % transform)
    rclpy.spin(node)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()

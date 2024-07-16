import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class EEFPoseListener(Node):
    '''
    The only purpose of this class is to listen to the TF and get the transform between the base and the end effector.
    Making it an independent node so it can be easily multi-threaded.
    '''
    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')  # type: ignore

        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)
    
    def get_transform(self, target_frame, source_frame):
        try:
            # Timeout is important so the TF can propely initialize instead of blocking the program with None indefinitely
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=2))
            return transform
        except Exception as e:
            self.get_logger().error('Failed to get transform: %s' % e)
            

def main():
    rclpy.init()
    eef_pose_node = EEFPoseListener()
    transform = eef_pose_node.get_transform('link_base', 'link_eef')

    rclpy.spin(eef_pose_node)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()

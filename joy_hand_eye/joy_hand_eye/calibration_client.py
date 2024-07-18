from gc import garbage
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from joy_hand_eye_msg.srv import CollectPoses
from std_srvs.srv import Empty


from usbcam_capture.usbcam_image_client import USBCamImageClient

import cv2
from cv_bridge import CvBridge

from queue import Queue
import os


class CalibrationClient(Node):

    def __init__(self):
        super().__init__('calibration_client') # type: ignore


        ############################ Miscanellous Setup #######################################
        self.cvbridge = CvBridge() # for converting ROS images to OpenCV images

        self._debounce_setup() # for debouncing the capture button
        self.shutter = False # when this is true, the client will issue a request to the server to capture images

        ############################ Client Setup #############################################
        # Pose collecting client
        # self.collect_pose_cli = self.create_client(
        #     srv_type=CollectPoses, 
        #     srv_name='/joy_hand_eye/collect_poses')
        # while not self.collect_pose_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('collect poses service not available, waiting again...')
        
        # self.pose_req = CollectPoses.Request()
    
    def _debounce_setup(self):
        '''
        As in any embedded system, we need to debounce the capture button.
        While we human think we press the button once, the computer actually consider the button pressed all the time during the duration of the press,
        because the polling rate is much faster than the human reaction time. 
        
        This function sets up a buffer to store the last value of the button press so that we can detect the rising edge.
        '''

        self.debounce_buffer = Queue(maxsize=1)
        self.debounce_buffer.put(0) # when nothing is pressed, the value is 0

def main():

    # construct the charuco board.

    rclpy.init()
    executor = MultiThreadedExecutor()

    # image_node = USBCamImageClient()
    # executor.add_node(image_node)

    client = CalibrationClient()
    executor.add_node(client)
    

    while rclpy.ok():
        # if image_node.shutter: # shutter down

        #     # send request to server to capture images
        #     image_future = image_node.image_cli.call_async(image_node.image_req)

        #     # immediately shutter up to debounce, so we don't caputre multiple images
        #     image_node.shutter = False
            
        #     # wait for the server to capture images
        #     rclpy.spin_until_future_complete(image_node, image_future)

        #     # get the images from the server
        #     image_response = image_future.result()

        #     # postprocess the image and get it
        #     image = image_node.postprocess(image_response.frame)
        #     # ros_image = image_node.cvbridge.cv2_to_imgmsg(image, encoding='bgr8')

        #     # client.pose_req.frame = ros_image
        #     # pose_future = client.collect_pose_cli.call_async(client.pose_req)
            
        #     client.get_logger().info('Pose collected...')
            
        # client.get_logger().info(f'The shutter is {image_node.shutter}')
        executor.spin_once()

    # image_node.destroy_node()
    client.destroy_node()
    rclpy.shutdown()
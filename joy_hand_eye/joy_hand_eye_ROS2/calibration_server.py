import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from joy_hand_eye_msg.srv import CollectPoses

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from usbcam_capture.usbcam_image_client import USBCamImageClient

import cv2
import yaml


class CalibrationServer(Node):
    '''
    An object solely responsible for detecting the charuco board in the image.
    This is not a ROS node. It's just more convenient to have it as a class to store all the variable that got reused.
    '''
    def __init__(self, config_path):
        super().__init__('calibration_server')  # type: ignore
        
        ############################ Charuco Setup ###################################
        # load the yaml file
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.img_size = (self.config['img_height'], self.config['img_width'])
        self.k = self.config['camera_matrix']
        self.d = self.config['distortion_coefficients']

        # construct the charuco board.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.charuco_board = cv2.aruco.CharucoBoard(
            size = (8, 6), # weirdly, this is columns, rows
            squareLength=0.021,
            markerLength=0.015,
            dictionary=self.aruco_dict)
        self.charuco_board.setLegacyPattern(True) # this is needed due to OpenCV's abrupt backward compatibility breakage

        # detector parameters
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.charuco_params = cv2.aruco.CharucoParameters()
        self.charuco_detector = cv2.aruco.CharucoDetector(
            board = self.charuco_board,
            charucoParams = self.charuco_params,
            detectorParams = self.detector_params,
        )
        
        self.all_obj_points = []
        self.all_img_points = []

        ############################ TF Setup ########################################
        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)

        self.all_gripper2base = []

        ############################ Service Setup ####################################
        self.image_service = self.create_service(
        srv_type=TakeImage, 
        srv_name='/usbcam_capture/get_image', 
        callback=self.usbcam_srv_callback)

    def detect_charuco_board(self, image):
        """
        Detect charuco board in image
        
        """
        charuco_corners, charuco_ids, marker_corners, marker_ids = self.charuco_detector.detectBoard(image)

        # render the detected board
        self.render_detected_board(image, charuco_corners, charuco_ids, marker_corners, marker_ids)
        
        obj_points, img_points = self.charuco_board.matchImagePoints(charuco_corners, charuco_ids)
        
        if len(obj_points) == 0 or len(img_points) == 0:
            print('ERROR: No charuco board found in image')
            return
        if len(obj_points) != len(img_points):
            print('ERROR: Number of object points and image points do not match')
            return
        
        self.all_obj_points.append(obj_points)
        self.all_img_points.append(img_points)

    def get_transform(self, target_frame, source_frame):
        try:
            # Timeout is important so the TF can propely initialize instead of blocking the program with None indefinitely
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=2))
            return transform
        except Exception as e:
            self.get_logger().error('Failed to get transform: %s' % e)
    
    
    def render_detected_board(self, image, charuco_corners, charuco_ids, marker_corners, marker_ids):
        """
        Render the detected charuco board on the image
        """
        aruco_drawn_img = cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)
        charuco_drawn_img = cv2.aruco.drawDetectedCornersCharuco(aruco_drawn_img, charuco_corners, charuco_ids, cornerColor=(0, 0, 255))
        
        # show the image and block until a key is pressed
        cv2.imshow('charuco board', charuco_drawn_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main():

    # construct the charuco board.

    charuco_seer = CharucoBoardSeer(config_path='/home/irving/Desktop/tactile_ws/src/joy_hand_eye/config/camera_config.yaml')
    rclpy.init()
    executor = MultiThreadedExecutor()

    eef_pose_node = EEFPoseListener()
    executor.add_node(eef_pose_node)

    image_node = USBCamImageClient()
    executor.add_node(image_node)

    while rclpy.ok():
        if image_node.shutter: # shutter down

            # send request to server to capture images
            image_future = image_node.image_cli.call_async(image_node.image_req)

            # immediately shutter up to debounce, so we don't caputre multiple images
            image_node.shutter = False
            
            # wait for the server to capture images
            rclpy.spin_until_future_complete(image_node, image_future)

            # get the images from the server
            image_response = image_future.result()

            # postprocess the image and get it
            image = image_node.postprocess(image_response.frame)
            charuco_seer.detect_charuco_board(image)

        executor.spin_once()



    transform = eef_pose_node.get_transform('link_base', 'link_eef')

    rclpy.spin(eef_pose_node)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()

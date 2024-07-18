import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from joy_hand_eye_msg.srv import CollectPoses
from std_srvs.srv import Empty
from sensor_msgs.msg import Image

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from transforms3d.quaternions import quat2mat

import cv2
from cv_bridge import CvBridge
import yaml
import numpy as np

class CalibrationServer(Node):
    '''
    An object solely responsible for detecting the charuco board in the image.
    This is not a ROS node. It's just more convenient to have it as a class to store all the variable that got reused.
    '''
    def __init__(self):
        super().__init__('calibration_server')  # type: ignore
        
        ############################ Charuco Setup ###################################
        # load the yaml file
        with open('/home/irving/Desktop/tactile_ws/src/joy_hand_eye_ROS2/joy_hand_eye/config/camera_config.yaml', 'r') as f:
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

        self.all_R_target2cam = []
        self.all_t_target2cam = []

        self.cvbridge = CvBridge() # for converting ROS images to OpenCV images

        ############################ TF Setup ########################################
        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)

        self.all_R_gripper2base = []
        self.all_t_gripper2base = []

        ############################ Service Setup ####################################
        self.my_callback_group = ReentrantCallbackGroup()

        # self.collect_pose_service = self.create_service(
        # srv_type=CollectPoses, 
        # srv_name='/joy_hand_eye/collect_poses', 
        # callback=self.collect_poses_callback,
        # callback_group=self.my_callback_group)

        self.hand_eye_service = self.create_service(
        srv_type=Empty, 
        srv_name='/joy_hand_eye/run_hand_eye_calibration', 
        callback=self.calibrate_callback,
        callback_group=self.my_callback_group)

        ############################ Subscriber Setup #################################
        self.image_sub = self.create_subscription(
            msg_type=Image, 
            topic='/usbcam_capture/captured_rgb_image', 
            callback=self.collect_poses_callback, 
            qos_profile=10,
            callback_group=self.my_callback_group)

    def collect_poses_callback(self, msg):
        image = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        success_flag = self.detect_charuco_board(image)

        if success_flag is not None:
            # get the transform from the gripper to the base
            gripper2base = self.get_transform(target_frame='link_base', source_frame='link_eef')
            self.process_tf(gripper2base)

        self.get_logger().info('Current number of images: %d' % len(self.all_obj_points))
        self.get_logger().info('Current number of poses: %d' % len(self.all_R_gripper2base))

    
    def calibrate_callback(self, request, response):
        # calibrate the hand-eye
        return response


    def detect_charuco_board(self, image):
        """
        Detect charuco board in image
        
        """
        charuco_corners, charuco_ids, marker_corners, marker_ids = self.charuco_detector.detectBoard(image)

        if charuco_corners is None or charuco_ids is None or marker_corners is None or marker_ids is None:
            self.get_logger().warning('No charuco board found in image. Take another image')
            return
        # render the detected board
        self.render_detected_board(image, charuco_corners, charuco_ids, marker_corners, marker_ids)
        
        obj_points, img_points = self.charuco_board.matchImagePoints(charuco_corners, charuco_ids)
        
        if len(obj_points) == 0 or len(img_points) == 0:
            self.get_logger().warning('No charuco board found in image. Take another image')
            return
        
        if len(obj_points) != len(img_points):
            self.get_logger().warning('Mismatch between object points and image points. Take another image')
            return
        
        self.all_obj_points.append(obj_points)
        self.all_img_points.append(img_points)
        
        return 1 # indicate success

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
    
    def process_tf(self, transformstamp):
        """
        Process the transform (TransformStamp) and return the translation and rotation both in numpy array
        """
        translation = np.array([transformstamp.transform.translation.x, transformstamp.transform.translation.y, transformstamp.transform.translation.z])
        quaternion = np.array([transformstamp.transform.rotation.x, transformstamp.transform.rotation.y, transformstamp.transform.rotation.z, transformstamp.transform.rotation.w])
        
        rotation = quat2mat(quaternion)

        self.get_logger().info(f'Current Gripper Rotation: {rotation}')
        self.get_logger().info(f'Current Gripper Translation: {translation}')
        
        self.all_R_gripper2base.append(rotation)
        self.all_t_gripper2base.append(translation)

def main():

    rclpy.init()
    server = CalibrationServer()

    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()

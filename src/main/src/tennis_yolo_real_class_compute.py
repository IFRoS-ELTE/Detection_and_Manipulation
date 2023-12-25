#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped # Import Pose message type for ball positions
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


# Import YOLO from Ultralytics
from ultralytics import YOLO


class BallDetector:
    def __init__(self):
        self.model = YOLO('./best2.pt')
        # Initialize CvBridge
        self.bridge1 = CvBridge()
        self.bridge2 = CvBridge()
        # self.bridge3 = CvBridge()
        self.ball_size = 65
        #Initializing camera parameters
        self.f = []
        self.K = []
        self.P = []
        self.dist_coeff = []
        self.point = []
        #initializing detected balls
        self.results=[]
        self.rgb_frame = []
        self.depth_frame = []
        self.pose3d = []
        self.compressed_depth_frame=[]

        self.T_camera_EE = np.array([[0,-1,0,0],
                              [1,0,0,-55],
                              [0,0,1,16],
                              [0,0,0,1]])
        # Subscribe to the RGB and Depth topics
        rgb_topic = '/camera/color/image_raw'
        depth_topic = '/camera/depth/image_rect_raw'
        compressed_depth_topic = '/camera/depth/image_rect_raw/compressed'
        camera_info_topic = '/camera/depth/camera_info'
        depth_camera_topic = '/camera/depth/color/points'

        # Initialize publisher for the processed image
        self.image_output_pub = rospy.Publisher("image_output", Image, queue_size=1)

        # Initialize publisher for ball positions as poses
        self.ball_positions_pub = rospy.Publisher("ball_positions", PoseStamped, queue_size=1)
        # Subscribe to the RGB and Depth topics
        rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)       
        rospy.Subscriber(rgb_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1, buff_size=2**24)
        

    def depth_to_3d(self, depth_img, camera_matrix, point, depth_units=0.001, color_img=None):
        """
        Converts a depth image and a color image from an RGBD camera into a colored point set.

        Parameters:
            - depth_img: the depth image (a Numpy array with elements of type uint16).
            - camera_matrix: the pinhole camera matrix with the intrinsics of the camera.
            - depth_units: the scale factor converting depth_image units (i.e., uint16) to meters.
            - color_img: the color image, aligned with the depth_image.
        Return:
            - pts_3d: 3D point set, a Numpy array of size num_pts x 3.
            - pts_colors: the RGB colors for the point set, a Numpy array of size num_pts x 3, values in the range 0..1.
        """
        point = (int(point[0]), int(point[1]))
        d = depth_img[point[1], point[0]]

        coords = np.hstack((point, 1)).reshape(1,3)
        coords = coords.T

        coords = np.linalg.inv(camera_matrix) @ coords
        norm = np.sqrt(coords[0]**2 + coords[1]**2 +coords[2]**2)
        coords = coords/norm
        coords = (coords  * d)

        pts_3d = coords.T


        return pts_3d
    
    def camera_info_callback(self, camera_info_msg):
        

        # Update camera intrinsic matrix and distortion coefficients
        self.f = camera_info_msg.K[0]
        self.K = camera_info_msg.K[:9]
        self.K=np.array(self.K).reshape(3,3)
        self.P = camera_info_msg.P
        self.dist_coeff = np.array(camera_info_msg.D)
        
        
    
    def depth_callback(self, depth_msg):
        
        # Convert ROS Image message to OpenCV image
        self.depth_frame = self.bridge2.imgmsg_to_cv2(depth_msg, '32FC1')  # Assuming 32-bit float depth values
        

    def image_callback(self, color_msg):
        
        # Convert ROS Image message to OpenCV image
        rgb = self.bridge1.imgmsg_to_cv2(color_msg, 'bgr8') 
        self.rgb_frame = rgb
        print('running')
        if self.rgb_frame!=[] and self.depth_frame!=[]:    
        # Use YOLO model on the RGB frame
            print('running model ..')
            results = self.model(rgb, stream=True, conf=0.5)
            for result in results:
                boxes = result.boxes

                for box in boxes:
                    # Bounding box
                    x1, y1, x2, y2 = box.xyxy[0]
                    print(f"Bounding box: ({x1}, {y1}) to ({x2}, {y2})")
                    x1_f = x1.item()
                    y1_f = y1.item()
                    # x2_f = x2.item()
                    # y2_f = y2.item()
                    

                    px = x1_f 
                    py = y1_f
                    self.point = np.array([px, py])
                    print('self.pose3d', self.pose3d)
                    if self.pose3d!=[]:
                        cv2.putText(rgb, f"X: {self.pose3d[0][0]:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.putText(rgb, f"Y: {self.pose3d[0][1]:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.putText(rgb, f"Z: {self.pose3d[0][2]:.2f}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        x1 = int(x1)
                        y1 = int(y1)
                        x2 = int(x2)
                        y2 = int(y2)
                        cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green rectangle
                    
                        

                    
            # Convert the processed image back to a ROS Image message
            image_msg = self.bridge1.cv2_to_imgmsg(rgb, encoding='bgr8')

            # Publish the processed image to the "image_output" rostopic
            self.image_output_pub.publish(image_msg)
            self.run()
            
        else:
            print('no pose')


    def distance(self, p1,p2):
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def run(self):
            
            if self.point!=[]: 
                print('pose before proj = ', self.point)              
                self.pose3d= self.depth_to_3d(self.depth_frame,self.K,self.point )
                print('pose after proj = ', self.pose3d)              
                
                pts_3d = np.hstack((self.pose3d[0],1 )).reshape(4,1) #homogeneous coordinates of the ball with respect to the camera
                p = self.T_camera_EE @ pts_3d
                self.pose3d = p.reshape(1,4)
                print('pose after trans = ', self.pose3d)
                pose = PoseStamped()
                pose.header.frame_id = 'link_eef'
                pose.pose.position.x = self.pose3d[0,0]
                pose.pose.position.y = self.pose3d[0,1]             
                pose.pose.position.z = self.pose3d[0,2]
                # # Publish the pose
                self.ball_positions_pub.publish(pose)
                

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('tennis_ball_detector')
    ball_detector_node = BallDetector()
    # Spin to keep the script alive
    rospy.spin()

    

    

    
        
    
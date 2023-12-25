#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped # Import Pose message type for ball positions
import rospy
import numpy as np
from std_msgs.msg import Header


# Import YOLO from Ultralytics
from ultralytics import YOLO

# Initialize ROS node
rospy.init_node('tennis_ball_detector')
global f
# Initialize YOLO model
model = YOLO('./best2.pt')

# Initialize CvBridge
bridge = CvBridge()

# Initialize publisher for the processed image
image_output_pub = rospy.Publisher("image_output", Image, queue_size=1)

# Initialize publisher for ball positions as poses
ball_positions_pub = rospy.Publisher("ball_positions", PoseStamped, queue_size=1)

def distance(p1,p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# Callback function to process incoming images and depth information
def image_callback(rgb_msg):
    global f, K, P
    try:
        # Convert ROS Image messages to OpenCV images
        rgb_frame = bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        # depth_frame = bridge.imgmsg_to_cv2(depth_msg, '32FC1')  # Assuming 32-bit float depth values

        # Use YOLO model on the RGB frame
        results = model(rgb_frame, stream=True, conf=0.5)

        for result in results:
            boxes = result.boxes

            for box in boxes:
                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                print(f"Bounding box: ({x1}, {y1}) to ({x2}, {y2})")

                # Calculate pose information based on bounding box
                pose_st = PoseStamped()
                pose_st.header.frame_id = 'link_eef'
                print('x1 =',x1)
                print('x2 =',x2)
                print('y1 =',y1)
                print('y2 =',y2)

                px = x1.item()
                py = y1.item()
                ball_pose = np.array([px, py, 1]).reshape(3,1)
                
                T = np.array([[0,-1,0,0],
                              [1,0,0,-55],
                              [0,0,1,16],
                              [0,0,0,1]])
                ball_pose_camera = (np.linalg.inv(K)@ball_pose)*1000
                # u_normalized= (px-K[0,2])/K[0,0]
                # v_normalized= (py-K[1,2])/K[1,1]
                print('ball_pose_camera', ball_pose_camera)
                
                focal = 1.88
                pixel_size = focal/f
                print('pixel_size=', pixel_size)
                print
                pz = (75*focal)/(x2*pixel_size)  # Use depth information for z-coordinate

             

                px = ball_pose_camera[0,0]
                py = ball_pose_camera[1,0]
                print('p before trans', px, ',', py, ',', pz.item())           
                
                
                p=np.array([px,py,pz.item(),1]).reshape(4,1)
                
                p = T@p         
                print('p after trans', p)           
                
                

                pose_st.pose.position.x = p[0,0]
                pose_st.pose.position.y = p[1,0]             
                pose_st.pose.position.z = p[2,0]
                # # Publish the pose
                ball_positions_pub.publish(pose_st)
                cv2.putText(rgb_frame, f"X: {pose_st.pose.position.x:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(rgb_frame, f"Y: {pose_st.pose.position.y:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(rgb_frame, f"Z: {pose_st.pose.position.z:.2f}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)
                cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green rectangle

        # Convert the processed image back to a ROS Image message
        image_msg = bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')

        # Publish the processed image to the "image_output" rostopic
        image_output_pub.publish(image_msg)

    except Exception as e:
        print(e)
def camera_info_callback(camera_info_msg):
    global f, K, P
    # Update camera intrinsic matrix and distortion coefficients
    f = camera_info_msg.K[0]
    K = camera_info_msg.K[:9]
    K=np.array(K).reshape(3,3)
    P = camera_info_msg.P
    
    
    
    
def main():
    global f, K, P
# Subscribe to the RGB and Depth topics
    rgb_topic = '/camera/color/image_raw'
    depth_topic = '/camera/depth/image_rect_raw'
    camera_info_topic = '/camera/color/camera_info'


    # Subscribe to the RGB and Depth topics
    rospy.Subscriber(camera_info_topic, CameraInfo, camera_info_callback)       
    rospy.Subscriber(rgb_topic, Image, image_callback, queue_size=1, buff_size=2**24)
    # rospy.Subscriber(depth_topic, Image, depth_callback, queue_size=1, buff_size=2**24)


    # Spin to keep the script alive
    rospy.spin()

    # Release the OpenCV window
    # cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from rclpy.node import Node
import rclpy
from collections import deque
from filterpy.kalman import KalmanFilter

#runde:   ros2 launch lane_following_cam example_bag.launch.py brightness:=125 saturation:=10 multiplier_bottom:=0.8 multiplier_top:=0.65 divisor:=9.0 cam_align:=-50
#f1tenth: ros2 launch lane_following_cam robot_compressed1.launch.py brightness:=-10 saturation:=10 multiplier_bottom:=1.0 multiplier_top:=0.65 divisor:=9.0
#munchen: ros2 launch lane_following_cam robot_compressed1.launch.py brightness:=-10 saturation:=10 multiplier_bottom:=1.0 multiplier_top:=0.45 divisor:=5.0
#jkk:     ros2 launch lane_following_cam robot_compressed1.launch.py multiplier_bottom:=1.0 multiplier_top:=0.65 divisor:=5.0 islane:=false

class LaneDetect(Node):
    def __init__(self):
        super().__init__('lane_detect')
        # parameters
        self.declare_parameter('raw_image', False)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('debug', True)
        self.declare_parameter('brightness', -10)
        self.declare_parameter('multiplier_bottom', 1.0)
        self.declare_parameter('multiplier_top', 0.45)
        self.declare_parameter('divisor', 3.0)
        self.declare_parameter('saturation', 10)
        self.declare_parameter('cam_align', 0)
        self.declare_parameter('islane', True)
        
        # Get parameter values
        self.brightness = self.get_parameter('brightness').value
        self.multiplier_bottom = self.get_parameter('multiplier_bottom').value
        self.multiplier_top = self.get_parameter('multiplier_top').value
        self.divisor = self.get_parameter('divisor').value
        self.saturation = self.get_parameter('saturation').value
        self.cam_align = self.get_parameter('cam_align').value
        self.islane = self.get_parameter('islane').value
        img_topic = self.get_parameter('image_topic').value
        if self.get_parameter('raw_image').value:
            self.sub1 = self.create_subscription(Image, img_topic, self.raw_listener, 10)
            self.sub1  # prevent unused variable warning
            self.get_logger().info(f'lane_detect subscribed to raw image topic: {img_topic}')
        else:
            self.sub2 = self.create_subscription(CompressedImage, '/image_raw/compressed', self.compr_listener, 10)
            self.sub2  # prevent unused variable warning
            self.get_logger().info(f'lane_detect subscribed to compressed image topic: {img_topic}')
        self.pub1 = self.create_publisher(Image, '/lane_img', 10)
        self.pub2 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.debug = self.get_parameter('debug').value
        # New publisher for lane center
        self.center_pub = self.create_publisher(Float32, '/lane_center', 10)

        self.kf = KalmanFilter(dim_x=2, dim_z=1)        # 2 state variables (position, velocity)
        self.kf.x = np.array([0, 0])                    # Initial state: [position, velocity]
        self.kf.F = np.array([[1, 1], [0, 1]])          # State transition matrix
        self.kf.H = np.array([[1, 0]])                  # Observation matrix
        self.kf.P *= 1000                               # Large initial uncertainty
        self.kf.R = 10                                  # Measurement noise (adjust for sensitivity). Higher value → less trust in measurements (smoother but slower)
        self.kf.Q = np.array([[0.01, 0], [0, 0.5]])     # Process noise (adjust for sensitivity). Higher value → faster response but can be more unstable

        # For extra center smoothing
        self.center_history = deque(maxlen=3)           # Adjust smoothness

        #Default values
        self.width = 640
        self.height = 480

    def raw_listener(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # print info of the image, only once not every time
        self.get_logger().info(f'First raw img arrived, shape: {cv_image.shape}', once=True)
        # Detect lanes
        lane_image = self.detect_lanes(cv_image)
        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(lane_image, 'bgr8')
        # Publish the image
        self.pub1.publish(ros_image)
    
    def compr_listener(self, msg):
        # Convert ROS CompressedImage message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # print info of the image, only once not every time
        self.get_logger().info(f'First compressed img arrived, shape: {cv_image.shape}', once=True)
        # Detect lanes
        lane_image = self.detect_lanes(cv_image)
        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(lane_image, 'bgr8')
        # Publish the image
        self.pub1.publish(ros_image)

    #Method to use, when dealing with lanes
    def lane_img(self, image):
        # Adjust brightness until lanes are visible
        imageBrightness = cv2.convertScaleAbs(image, alpha=1, beta=self.brightness)
        
        hsv = cv2.cvtColor(imageBrightness, cv2.COLOR_BGR2HSV)       #hsv doesnt work on runde mcap
        
        # Recognise lanes based on color (white, yellow)
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, self.saturation, 255], dtype=np.uint8)      #change saturation, so it only recognises lanes
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        lower_yellow = np.array([20, 200, 100], dtype=np.uint8)
        upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask = cv2.bitwise_or(mask_white, mask_yellow)
        filtered_image = cv2.bitwise_and(image, image, mask=mask)
        
        # Convert to grayscale
        gray = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)
        # Detect edges using Canny
        edges = cv2.Canny(gray, 75, 150)
        # Defining ROI
        self.height, self.width = edges.shape
        mask = np.zeros_like(edges)
        
        polygon = np.array([[
            (0, int(self.height * self.multiplier_bottom)),          #runde: 0.8     other: 1    
            (self.width, int(self.height * self.multiplier_bottom)),
            (self.width, int(self.height * self.multiplier_top)),              #big_track_munchen_only_camera_a.mcap: 0.45 f1tenth: 0.6 runde: 0.65
            (0, int(self.height * self.multiplier_top))
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        return cropped_edges

    def tube_img(self, image):
        # Greyscale edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.convertScaleAbs(gray, alpha=10, beta=20)
        gray = cv2.GaussianBlur(gray, (15, 19), 0)

        edges = cv2.Canny(gray, 0, 50)
        
        # Defining ROI
        self.height, self.width = edges.shape
        mask = np.zeros_like(edges)
        
        polygon = np.array([[
            (0, int(self.height * self.multiplier_bottom)),
            (self.width, int(self.height * self.multiplier_bottom)),
            (self.width, int(self.height * self.multiplier_top)),
            (0, int(self.height * self.multiplier_top))
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        # Post-process edges to close gaps and reduce noise
        kernel = np.ones((3, 3), np.uint8)
        final_edges = cv2.morphologyEx(cropped_edges, cv2.MORPH_CLOSE, kernel)
        
        return final_edges

    def detect_lanes(self, image): 
        
        # Check if the image is a tube image
        # Detect lines using Hough transform
        
        if self.islane:
            lines = cv2.HoughLinesP(self.lane_img(image), 1, np.pi / 180, 50, maxLineGap=50)
        else:
            lines = cv2.HoughLinesP(self.tube_img(image), 1, np.pi / 180, 50, maxLineGap=50)

        line_image = np.zeros_like(image)
        left_x = []
        right_x = []

        if (len(self.center_history) == 0) or (self.center_history[-1] in range(-self.width, 2 * self.width)):
            check_center = self.width / 2                       #Initializing a default value for center if the deque is empty
        else:
            deque_center = np.mean(self.center_history)         #If not, we calculate a center using the last few cennters
            
            self.kf.predict()                                   #And we predict
            self.kf.update(np.array([deque_center]))
            check_center = self.kf.x[0]
            

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                if length > 60:                                                     # Only accepting lines that measure over 60 px in length
                    slope = (y2 - y1) / (x2 - x1) if x2 != x1 else np.inf
                    if 0.1 < abs(slope) < 5.0:  # Filter based on slope
                        cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 5)
                        if x1 < check_center and x2 < check_center:                 # If the turn is so steep, that it enters the other side of the screen, we account for it here
                            if slope < 0.2:
                                x1 += self.width/(2*self.divisor)
                                x2 += self.width/(2*self.divisor)
                            left_x.extend([x1, x2])
                        elif x1 > check_center and x2 > check_center:
                            if slope < 0.2:
                                x1 -= self.width/(2*self.divisor)
                                x2 -= self.width/(2*self.divisor)
                            right_x.extend([x1, x2])

        # Calculating the center
        center = self.width / 2          # Default center

        if left_x and right_x:
            left_avg = np.mean(left_x)
            right_avg = np.mean(right_x)
            center = (left_avg + right_avg) / 2
        elif left_x:
            left_avg = np.mean(left_x)
            center = (left_avg + self.width) / 2
            center = center + ((self.width/self.divisor) * (5 * (1 - (self.divisor/10))) * (0.5 - (abs(center - left_avg)/self.width)))    # Adjust divisor to lane size
        elif right_x:
            right_avg = np.mean(right_x)
            center = (right_avg - (self.width/self.divisor))/ 2
            center = center - ((self.width/self.divisor) * (5 * (1 - (self.divisor/10))) * (0.5 - (abs(center - right_avg)/self.width)))   # Adjust divisor to lane size

        # Averaging the last few measured centers
        self.center_history.append(center)
        deque_center = np.mean(self.center_history)

        # Applying the Kalman filter
        self.kf.predict()  # Prediction step
        self.kf.update(np.array([deque_center]))        # Update with the new measurement
        smoothed_center = self.kf.x[0]                  # Get the filtered (smoothed) center position

        # Twist logic

        twist = Twist()

        if not left_x and not right_x:
            # If there are no lines detected, slow the robot
            twist.angular.z = 0.0
            twist.linear.x = 0.05
            self.pub2.publish(twist)
        else:
            twist.linear.x = 0.2
            #   velocity (m/s)
            twist.angular.z = -0.0025 * ((smoothed_center + self.cam_align) - (self.width/2))
            #   angular velocity(rad/s)       turn left -> positive value, turn right -> negative value
            self.pub2.publish(twist)

        # Displaying center of lane using smoothed center
        cv2.line(line_image, (int((smoothed_center)), self.height), (int((smoothed_center)), int(self.height * (self.multiplier_top + 0.1))), (255, 0, 0), 2)
        cv2.circle(line_image, (int((smoothed_center)), int(self.height * self.multiplier_top)), 5, (255, 0, 0), -1)

        # Display the twist.angular.z value on the image and direction (left or right or straight)

        font = cv2.FONT_HERSHEY_SIMPLEX
        if twist.angular.z > 0.01:
            text = 'Left'
        elif twist.angular.z < -0.01:
            text = 'Right'
        else:
            text = 'Straight'
        cv2.putText(line_image, f'{text} {abs(twist.angular.z):.2f}', (10, 30), font, 1, (60, 40, 200), 2, cv2.LINE_AA)
        
        # Combine the original image with the line image
        if self.debug:
            combined_image = cv2.addWeighted(image, 0.3, line_image, 1, 1)
        else: 
            combined_image = line_image

        # Publish the center as a Float32 message
        center_msg = Float32()
        center_msg.data = smoothed_center
        self.center_pub.publish(center_msg)

        return combined_image

def main(args=None):
    rclpy.init(args=args)
    lane_detect = LaneDetect()
    rclpy.spin(lane_detect)
    lane_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
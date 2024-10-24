import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetect(Node):
    def __init__(self):
        super().__init__('lane_detect')
        # self.sub1 = self.create_subscription(Image, '/image_raw', self.raw_listener, 10)
        # self.sub1  # prevent unused variable warning
        self.sub2 = self.create_subscription(CompressedImage, '/image_raw/compressed', self.compr_listener, 10)
        self.sub2  # prevent unused variable warning
        self.pub1 = self.create_publisher(Image, '/lane_img', 10)
        self.bridge = CvBridge()

    def raw_listener(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # print info of the image, only once not every time
        self.get_logger().info(f'First raw img arrived, shape: {cv_image.shape}', once=True)
    
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

    def detect_lanes(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # Detect edges using Canny
        edges = cv2.Canny(blur, 50, 150)
        # Define a region of interest
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height),
            (width, height),
            (width, int(height * 0.6)),
            (0, int(height * 0.6))
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(cropped_edges, 1, np.pi / 180, 50, maxLineGap=50)
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
        # Combine the original image with the line image
        combined_image = cv2.addWeighted(image, 0.8, line_image, 1, 1)
        return combined_image

def main(args=None):
    rclpy.init(args=args)
    lane_detect = LaneDetect()
    rclpy.spin(lane_detect)
    lane_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
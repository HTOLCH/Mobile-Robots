import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from depthai_sdk import OakCamera

class Camera(Node):
    def __init__(self):
        super().__init__('camera')

        # Initialize CvBridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Create a publisher for raw image topic
        self.publisher_ = self.create_publisher(Image, 'raw_image', 10)

        # Create the OakCamera instance
        with OakCamera() as oak:
            self.color = oak.create_camera('color', resolution='1080p', encode='jpeg', fps=30)

            # Start capturing frames from the camera
            oak.start(blocking=False)

        # Create a timer to periodically capture frames and publish them
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Camera node has been started.')

    def timer_callback(self):
        # Capture a frame from the camera
        color_frame = self.color.get_frame()

        if color_frame is not None:
            # Convert the frame (OpenCV) to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")

            # Publish the ROS Image message
            self.publisher_.publish(ros_image)

        else:
            self.get_logger().warn("No frame captured!")

def main(args=None):
    rclpy.init(args=args)
    node = Camera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

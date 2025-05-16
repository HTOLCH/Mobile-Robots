import rclpy
from rclpy.node import Node
from depthai_sdk import OakCamera


with OakCamera() as oak:
    color = oak.create_camera('color', resolution='1080p', encode='jpeg', fps=30)

    oak.start(blocking=True)


class Camera(Node):
    def __init__(self):
        super().__init__('camera')

        self.publisher_ = self.create_publisher(IMG, 'raw_image', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Camera node has been started.')


    def timer_callback(self):
        self.publisher_.publish(self.color)

        

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

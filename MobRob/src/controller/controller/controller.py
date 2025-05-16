import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
#Run: 
# ros2 run joy_linux joy_linux_node
#To publish joystick information



class JoyListenerNode(Node):
    def __init__(self):
        super().__init__('joy_listener')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to the 'joy' topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.previous_buttons = []  # Track previous button states
        self.get_logger().info('JoyListenerNode has been started.')

        self.current_twist = Twist()

        self.manual_mode = True
        self.automatic_mode = False


    def joy_callback(self, msg):
        
        
        #self.current_twist.linear.x = msg.axes[1]  # Left joystick vertical (forward/backward)
        #self.current_twist.angular.z = msg.axes[0]
        # Assuming the forward movement corresponds to the Y-axis (msg.axes[1])
        #threshold = 0.5  # Define the threshold for detecting forward movement
        #if msg.axes[1] > threshold:
            #self.get_logger().info("Joystick moved forward!")
        #self.get_logger().info(f"x axis = {msg.axes[2]}\ny axis = {msg.axes[3]}")

        if msg.buttons[7] == 1:
            self.get_logger().info(f"X Axes: {-msg.axes[0]}")
            self.get_logger().info(f"Y Axes: {msg.axes[1]}")
            self.get_logger().info(f"fine_X Axes: {-msg.axes[7]}")
            self.get_logger().info(f"fine_Y Axes: {msg.axes[6]}")
            forward = msg.axes[1] + (msg.axes[7]/4) #Allow for fine forward turning with left buttons
            turn = msg.axes[0] + (msg.axes[6]/4)  #Allow for fine turn turning with left buttons
            self.current_twist.linear.x = forward  
            self.current_twist.angular.z = turn
        else:
            self.current_twist.linear.x = float(0)  # Left joystick vertical (forward/backward)
            self.current_twist.angular.z = float(0)
        
        # Initialize previous_buttons to match the number of buttons on the joystick
        if not self.previous_buttons:
            self.previous_buttons = [0] * len(msg.buttons)
        
        for i, button_state in enumerate(msg.buttons):
            # Check for a button press-and-release event
            if self.previous_buttons[i] == 1 and button_state == 0:
                if i == 0:
                    self.get_logger().info("Entering Automated Mode")
                if i == 1:
                    self.get_logger().info("Entering Manual Mode")
                #self.get_logger().info(f"Button {i} was pressed and released.")
        
        # Update the previous_buttons to the current state
        self.previous_buttons = msg.buttons

        self.publisher_.publish(self.current_twist)       



def main(args=None):
    rclpy.init(args=args)
    node = JoyListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

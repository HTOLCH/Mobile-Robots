import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
import math
from time import sleep
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray  # For bounding box coordinates
import time
from interfaces.srv import TakePhoto
from std_msgs.msg import String
import csv

#Master class
class autoDrive(Node):

    #initialize paremeters
    def __init__(self):
        super().__init__('autoDrive')

        self.auto_cmd_publisher = self.create_publisher(Twist, 'auto_joy', 10)
        self.distances_publisher = self.create_publisher(String, 'distances', 10)
        self.avoidance_status_publisher = self.create_publisher(String, 'avoidance_status', 10)
        self.operation_state_publisher = self.create_publisher(String, 'operation_state', 10)
        self.cbDistances_publisher = self.create_publisher(Float32MultiArray, 'cbDistances', 10)

        # Create a client for the 'take_photo' service
        self.camera_client = self.create_client(TakePhoto, 'take_photo')

        # Subscriber to the 'auto_status' topic
        self.auto_status_subscription = self.create_subscription(
            Bool,
            'auto_status',
            self.auto_status_callback,
            10
        )

        # Subscriber to the 'phidget' topic
        self.phidget_subscription = self.create_subscription(
            Float64,
            'phidget',
            self.phidget_callback,
            10
        )

        # Subscriber to the 'scan' topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Subscriber to the 'fix' topic
        self.fix_subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        # Subscriber to the 'bounding_boxes' topic
        self.bb = self.create_subscription(
            Float32MultiArray ,
            '/bounding_boxes',
            self.bounding_boxes_callback,
            10
        )

        # Subscriber to the 'bounding_boxes' topic
        self.bb = self.create_subscription(
            Float32MultiArray ,
            '/bounding_boxes_object',
            self.bounding_boxes_object_callback,
            10
        )

        self.get_logger().info('autoDrive has been started.')

        self.current_twist = Twist()
        
        self.activated = False

        self.heading = 0

        self.bounding_box = None

        self.bounding_box_object = None

        "Operation states: "
        "T = Travelling to waypoint"
        "C = Cone operations"
        "B = Bucket operations"
        "S = Storing values"
        "L = Leaving waypoint"

        self.operation_state = "T"

        self.cone_distances = []

        self.bucket_distances = []

        self.cbDistances = []

        self.current_cone_distance = 0

        self.current_bucket_distance = 0

        self.waypoint_heading = 0

        self.smallest_waypoint_distance = 100

        self.object_heading = 0
        
        self.smallest_object_distance = 100

        self.center_count = 0

        self.avoidance_status = "No objects detected"

        self.gps_valid = False

        self.min_right = None
        self.min_forward = None
        self.min_left = None
        
        self.avoidance = False

        self.was_avoiding = False

        self.leaving_waypoint = False

        self.centered = False

        self.startup = True

        self.first_turn = True
        
        self.final_heading_rad = 0

        self.photo_taken = False

        self.starting_lat = None

        self.starting_long = None

        self.on_final_approach = False

        self.distances = ""

        self.target_lat = None

        self.target_long = None

        # Initialize current position and heading
        self.current_lat = 0.0
        self.current_lon = 0.0

        # Waypoints (latitude, longitude)
        #self.waypoints = [
        #    (-31.980534, 115.8177542),  # Waypoint 1
        #    (-31.98044, 115.81756),  # Waypoint 2
            # Add more waypoints here
        #]
        gps_filepath = "/workspace/GPS_Waypoints/waypoints.csv"
        self.waypoints = []
        with open(gps_filepath, newline='') as csvfile:
            waypoint_reader = csv.reader(csvfile)
            for row in waypoint_reader:
                # Convert the string values to float and add to the list
                lat, lon = float(row[0]), float(row[1])
                self.waypoints.append((lat, lon))

        self.current_waypoint_idx = 2

        sleep(5)

        self.get_logger().info(str(self.waypoints))

        #self.timer = self.create_timer(0.5, self.collision_avoidance)

        

        # Wait for the service to become available
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for camera service...')

        self.get_logger().info('Waiting for lidar values...')

        #self.timer = self.create_timer(0.5, self.find_object("cone"))
        #self.timer = self.create_timer(0.1, lambda: self.leave_waypoint())

        self.timer = self.create_timer(0.1, self.current_mode_publisher)
        self.timer = self.create_timer(0.1, self.collision_avoidance)

    def cbDistances_publisher_function(self):
        message = Float32MultiArray()
        message.data = self.cbDistances
        self.cbDistances_publisher.publish(message)
        
    def current_mode_publisher(self):
        message = String()
        message.data = self.operation_state
        if self.operation_state == "CC":
            message.data = "C"
        #message.data = "B"
        #self.get_logger().info(f"Current operation state: {self.operation_state}")
        self.operation_state_publisher.publish(message)


    def phidget_callback(self,msg):
        self.heading = msg.data
        #print("yes")
        #self.get_logger().info(f"IMU Heading: {msg.data:.2f} degrees")

    def auto_status_callback(self, msg):

        if msg.data == True:
            self.activated = True
        else:
            self.activated = False

    def bounding_boxes_callback(self, msg):
        self.bounding_box = msg.data
        #self.get_logger().info(str(self.bounding_box))
        if self.bounding_box:
            self.bounding_box = tuple(self.bounding_box)
        else:
            self.bounding_box = None

    def bounding_boxes_object_callback(self, msg):
        self.bounding_box_object = msg.data
        #self.get_logger().info(str(self.bounding_box))
        if self.bounding_box_object:
            self.bounding_box_object = tuple(self.bounding_box_object)
        else:
            self.bounding_box_object = None
    
    def collision_avoidance(self):
        # Get distances from sensors
        left = self.min_left
        front = self.min_forward
        right = self.min_right

        # Initialize the avoidance flag and control variables
        clear = False
        avoidance_speed = 1.0  # Start at max speed
        angular_speed = 0.0  # No turning by default

        # Check for objects in front, left, and right
        if front < 1:
            self.avoidance_status = "Object in front"
            # Slow down the robot if an obstacle is too close
            avoidance_speed = -0.4
            if left > right:
                angular_speed = 0.7  # Turn left
            else:
                angular_speed = -0.7  # Turn right

        elif left < 0.5 and front > 0.5 and right > 0.5:
            self.avoidance_status = "Object to the left"
            # Object is to the left, move right
            avoidance_speed = 0.5
            angular_speed = -0.6

        elif right < 0.5 and front > 0.5 and left > 0.5:
            self.avoidance_status = "Object to the right"
            # Object is to the right, move left
            avoidance_speed = 0.5
            angular_speed = 0.6
            
        elif left < 0.5 and front < 0.5 and right < 0.5:
            # If the robot is surrounded, stop and turn in place
            self.avoidance_status = "I'm surrounded!"
            avoidance_speed = 0
            angular_speed = 0
        else:
            # If the coast is clear, move forward
            self.avoidance_status = "Coast is clear"
            clear = True

        # Publish movement commands and status
        if clear:
            if self.was_avoiding:
                self.get_logger().info("Path is now clear, resume normal operation...")
                self.avoidance = False
                self.current_twist.linear.x = float(0)
                self.current_twist.angular.z = float(0)
                self.auto_cmd_publisher.publish(self.current_twist)
                self.was_avoiding = False
            
        else:
            self.avoidance = True
            self.current_twist.linear.x = float(avoidance_speed)
            self.current_twist.angular.z = float(angular_speed)
            self.auto_cmd_publisher.publish(self.current_twist)
            self.was_avoiding = True

        #Publish the minimum distance
        #self.get_logger().info(self.avoidance_status)
        message = String()
        message.data = self.avoidance_status
        self.avoidance_status_publisher.publish(message)

 
    def scan_callback(self, msg):
        
        self.cbDistances_publisher_function()

        def valid_ranges(data):
            return [r for r in data if r < 15.0 and r > 0.0]

        min_range = Float64()

        # Example manipulation: find minimum range
        if len(msg.ranges) > 0:
            filtered = valid_ranges(msg.ranges)
            if filtered:
                min_range.data = min(msg.ranges)
            else:
                self.get_logger().info('All ranges are invalid')
                return
            #self.get_logger().info(f'Minimum detected range: {min_range:.2f} meters')
        else:
            self.get_logger().info('No range data received')
        
        try:
            regions = {
                'right': msg.ranges[320:600],
                'forward': msg.ranges[600:840],
                'left': msg.ranges[840:1120]
            }

            # Find minimum in each region
            self.min_right = min(regions['right'])
            self.min_forward = min(regions['forward'])
            self.min_left = min(regions['left'])

        except ValueError as e:
            self.get_logger().error(f"Error processing ranges: {e}")
            return


        self.distances = f"Left: {self.min_left:.2f} | Forward: {self.min_forward:.2f} | Right: {self.min_right:.2f}"
        # Publish the distances
        message = String()
        message.data = self.distances
        self.distances_publisher.publish(message)
        

        #self.get_logger().info(
        #   f"Min distances - Right: {self.min_right:.2f} | Forward: {self.min_forward:.2f} | Left: {self.min_left:.2f}"
        #)

        #if self.activated: #Only run avoidance when on automatic mode
        #    if self.completed_waypoint_operations: #Ensures that collision avoidance will only run while driving between waypoints.
        #        if not self.gps_not_valid: #Ensures that we are recieving a gps signal, allowing the pathfinding code to run.
        #            self.collision_avoidance()
        

        #self.get_logger().info(str(len(msg.ranges)))
        #self.get_logger().info(str(msg.intensities))

        

    def gps_callback(self, msg: NavSatFix):
        # Check if the latitude and longitude are valid

        #self.get_logger().info(f"Cone Distances : {self.cone_distances}")
        #self.get_logger().info(f"Bucket Distances : {self.bucket_distances}")
        #self.get_logger().info(f"Distance between cone and bucket: {self.cbDistances}")

        if self.activated:            
            if self.operation_state == "T": #Travelling to waypoint
                if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):

                    if self.starting_lat is None and self.starting_long is None:
                        self.starting_lat = msg.latitude
                        self.starting_long = msg.longitude

                    self.get_logger().info(f"Starting lat, long: ({self.starting_lat}, {self.starting_long})")

                    self.gps_valid = True
                    self.current_lat = msg.latitude
                    self.current_lon = msg.longitude
                    #self.get_logger().info(f"Received valid GPS: ({self.current_lat}, {self.current_lon})")
                    if not self.avoidance: #Only drive to waypoint if we are not currently avoiding an object.
                        self.drive_to_waypoint()  # Only proceed if valid GPS data is available
                else:
                    self.get_logger().warn("Invalid GPS data received. Waiting for valid data...")
                    self.gps_valid = False

            #elif self.operation_state == "C": #Cone operations
            #    self.find_waypoint()
            #Updated version
            elif self.operation_state == "C": #Cone operations
                self.find_object()

            #elif self.operation_state == "B": #Bucket operations
            #    self.find_bucket()
            #Updated version
            elif self.operation_state == "CC":
                self.find_object()

            elif self.operation_state == "B": #Bucket operations
                self.find_object()

            elif self.operation_state == "S": #Store values
                self.store_values()

            elif self.operation_state == "L": #Leaving waypoint
                #self.get_logger().info("Leaving waypoint...")
                self.leave_waypoint()
        


    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # Calculate the great-circle distance between two points
        # Use Haversine formula or any other suitable method
        R = 6371.0  # Earth radius in km
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance  # in kilometers

    def calculate_heading(self, lat1, lon1, lat2, lon2):
        # Calculate the bearing between two GPS coordinates (in radians)
        dlon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)

        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        heading = math.atan2(x, y)
        return heading

    def normalize_angle(self, angle):
        # Normalize angle to be within [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


    def find_object(self):

        if self.operation_state == "L":
            self.current_twist.angular.z = float(0) 
            self.current_twist.linear.x = float(0)
            self.auto_cmd_publisher.publish(self.current_twist)
            return


        self.get_logger().info(f"Waypoint Heading:  {math.degrees(self.waypoint_heading)}")
        self.get_logger().info(f"Object Heading:    {math.degrees(self.object_heading)}")
        self.get_logger().info(f"Waypoint Distance: {self.current_cone_distance}")
        self.get_logger().info(f"Object Distance:   {self.current_bucket_distance}")
    

        if self.first_turn:
            self.get_logger().info(f"Finding {self.operation_state}...")
            self.final_heading_rad = math.radians(self.heading - 30)
            self.first_turn = False

        #Spin in a 330 degree circle  
        current_heading_rad = math.radians(self.heading)
        heading_error_rad = self.normalize_angle(self.final_heading_rad - current_heading_rad)
        self.get_logger().info(f"Current Heading: {math.degrees(current_heading_rad)}")
        self.get_logger().info(f"Desired Heading: {math.degrees(self.final_heading_rad)}")
        self.get_logger().info(f"Heading error: {heading_error_rad}")

        #See if we are have completed a full rotation
        if abs(heading_error_rad) > 0.10:
            #If not, turn right.
            self.current_twist.angular.z = float(-0.15) 
            self.current_twist.linear.x = float(0)
        
        else:
            #Turn complete
            self.current_twist.angular.z = float(0) 
            self.current_twist.linear.x = float(0)

            if self.operation_state == "B":
                self.operation_state = "S" #Store values

                self.first_turn = True
                
                return
             

            if self.operation_state == "C":
                self.operation_state = "CC" #Change operation state to cone operations         

            return

        center_tolerance = 35

        #See if there is an object in the bounding box
        if self.bounding_box and self.operation_state == "C":
            self.get_logger().info(f"smallest waypoint distance: {self.smallest_waypoint_distance}")
            #self.current_twist.angular.z = float(-0.1) 
            x1, y1, x2, y2 = self.bounding_box
            #self.get_logger().info(f"Received Cone Bounding Box: {self.bounding_box}")
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            image_center_x = 600
            diff_x = image_center_x - center_x
            diagonal_length = ((x2-x1)**2 + (y2-y1)**2)**0.5
            #self.get_logger().info(f"Diff x: {diff_x}")

            if (abs(diff_x) < 1000):
                self.current_twist.angular.z = float(min( max(-abs(diff_x)/3000,-0.04) ,-0.02)) 
            

            if (abs(diff_x) < center_tolerance) and (self.min_forward < self.smallest_waypoint_distance):
                #The waypoint heading needs to be set as the current heading
                self.get_logger().info(f"Found Waypoint!")
                self.waypoint_heading = math.radians(self.heading)
                #NEW
                self.smallest_waypoint_distance = self.min_forward

        if self.operation_state == "CC":
            #Turn back to the cone and move until within 2m

            heading_rad = math.radians(self.heading)

            desired_heading_rad = (self.waypoint_heading)
        
            self.get_logger().info(f"Current Heading: {math.degrees(heading_rad)}")
            self.get_logger().info(f"Desired Heading: {math.degrees(desired_heading_rad)}")
            

            heading_error_rad = self.normalize_angle(desired_heading_rad - heading_rad)
            #self.get_logger().info(f"Heading error: {heading_error_rad}")


            if heading_error_rad < -0.02:
                #self.get_logger().info(f"Turning left...")
                self.current_twist.angular.z = float(min(-heading_error_rad, 0.5))
                self.current_twist.linear.x = float(0)

            elif heading_error_rad > 0.02:
                #self.get_logger().info(f"Turning right...")
                self.current_twist.angular.z = float(max(-heading_error_rad, -0.5)) 
                self.current_twist.linear.x = float(0)
            
            else:
                self.get_logger().info(f"Reached heading.")
                self.current_twist.angular.z = float(0) 
                #self.current_twist.linear.x = float(0)

                self.current_cone_distance = self.min_forward

                #Move until within 2m of the cone
                if self.current_cone_distance < 3.5:
                    self.current_twist.angular.z = float(0) 
                    self.current_twist.linear.x = float(0)
                    self.auto_cmd_publisher.publish(self.current_twist)
                    self.get_logger().info(f"Reached desired cone distance, taking photo...")
                    sleep(2)
                    self.take_photo()
                    self.operation_state = "B" #Change operation state to bucket operations 
                    self.first_turn = True
                    return

                else:
                    self.current_twist.linear.x = float(0.5)
                    self.current_twist.angular.z = float(0)                         

            self.auto_cmd_publisher.publish(self.current_twist)
        
            

        #See if there is a bucket bounding box
        if self.bounding_box_object and self.operation_state == "B":
            #self.current_twist.angular.z = float(-0.1) 
            x1, y1, x2, y2 = self.bounding_box_object
            #self.get_logger().info(f"Received Object Bounding Box: {self.bounding_box_object}")
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            image_center_x = 600
            diff_x = image_center_x - center_x
            diagonal_length = ((x2-x1)**2 + (y2-y1)**2)**0.5
            #self.get_logger().info(f"Diff x: {diff_x}")
            
            if (abs(diff_x) < 1000):
                self.current_twist.angular.z = float(min( max(-abs(diff_x)/3000,-0.04) ,-0.02)) 


            if (abs(diff_x) < center_tolerance) and (self.min_forward < self.smallest_object_distance):
                #The waypoint heading needs to be set as the current heading
                self.get_logger().info(f"Found Object!")
                self.object_heading = math.radians(self.heading)
                #NEW
                self.smallest_object_distance = self.min_forward

                self.current_bucket_distance = self.min_forward
                if self.photo_taken == False:
                    self.take_photo()
                    #self.photo_taken = True
        

        self.auto_cmd_publisher.publish(self.current_twist)

        #sleep(5)


    def store_values(self):
        #Store current distances to cone and bucket lists
        self.cone_distances.append(self.current_cone_distance)
        self.bucket_distances.append(self.current_bucket_distance)

        #Find the distance between the cone and bucket using cosine rule
        angle_between = self.waypoint_heading - self.object_heading
        cbDistance = math.sqrt(self.current_cone_distance**2 + self.current_bucket_distance**2 - 2 * self.current_cone_distance * self.current_bucket_distance * math.cos(angle_between))
        self.get_logger().info(f"Distance between cone and bucket: {cbDistance}")

        #Append to the list
        self.cbDistances.append(cbDistance)

        self.current_waypoint_idx += 1

        self.operation_state = "L" #Change operation state to leaving waypoint


       
    def leave_waypoint(self):

        self.get_logger().info(f"Leaving waypoint...")
        #sleep(2)
        #Face 90 degrees to the left of the waypoint
        #self.waypoint_heading = math.radians(100)
        heading_rad = math.radians(self.heading)

        desired_heading_rad = (self.waypoint_heading - (math.pi/2))
      
        self.get_logger().info(f"Current Heading: {math.degrees(heading_rad)}")
        self.get_logger().info(f"Desired Heading: {math.degrees(desired_heading_rad)}")
        

        heading_error_rad = self.normalize_angle(desired_heading_rad - heading_rad)
        #self.get_logger().info(f"Heading error: {heading_error_rad}")


        if heading_error_rad < -0.02:
            #self.get_logger().info(f"Turning left...")
            self.current_twist.angular.z = float(min(-heading_error_rad, 0.5))
            self.current_twist.linear.x = float(0)

        elif heading_error_rad > 0.02:
            #self.get_logger().info(f"Turning right...")
            self.current_twist.angular.z = float(max(-heading_error_rad, -0.5)) 
            self.current_twist.linear.x = float(0)
        
        else:
            self.get_logger().info(f"Reached heading.")
            self.current_twist.angular.z = float(0) 
            self.current_twist.linear.x = float(0)

            #Change operation state to travelling to next waypoint
            self.operation_state = "T"
            self.leaving_waypoint = True
            

        self.auto_cmd_publisher.publish(self.current_twist)


    def take_photo(self):
        # Create a request (empty if no parameters are required)
        request = TakePhoto.Request()

        # Call the service asynchronously and handle the result
        future = self.camera_client.call_async(request)
        future.add_done_callback(self.handle_take_photo_response)

    def handle_take_photo_response(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f"Photo taken successfully: {response.message}")
        else:
            self.get_logger().warn(f"Failed to take photo: {response.message}")

    def drive_to_waypoint(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            if not self.on_final_approach:
                self.get_logger().info("All waypoints reached, driving to home position...")
                self.target_lat = self.starting_lat
                self.target_long = self.starting_long

                self.on_final_approach = True
        else:
            # Get the next waypoint
            self.target_lat, self.target_long = self.waypoints[self.current_waypoint_idx]

        self.get_logger().info(f"Current Waypoint Index: {self.current_waypoint_idx}")
        
        # Calculate the distance to the target (Haversine formula or simpler method)
        distance = self.calculate_distance(self.current_lat, self.current_lon, self.target_lat, self.target_long)
        self.get_logger().info(f"Distance to point: {distance}")

        # If the robot is close enough to the waypoint, do waypoint operations
        if distance < 0.002 and self.operation_state == "T":
            # If we are on the final approach, stop the robot
            if self.on_final_approach:
                self.operation_state = "F"
                self.get_logger().info("Reached home position, stopping...")
                return
            
            #Only do this if we are within waypoint range and are still in the travelling state
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx}, locating waypoint marker...")
            self.operation_state = "C" #Change operation state to cone operations
            
            #Reset waypoint variables
            self.smallest_object_distance = 100
            self.smallest_waypoint_distance = 100
            self.object_heading = 0
            self.waypoint_heading = 0
            self.current_cone_distance = 0
            self.current_bucket_distance = 0
            
            return
        

        # Calculate the desired heading towards the next waypoint
        desired_heading = self.calculate_heading(self.current_lat, self.current_lon, self.target_lat, self.target_long)
        self.get_logger().info(f"Desired Heading: {math.degrees(desired_heading)}")
        
        self.heading = math.radians(self.heading)
        self.get_logger().info(f"Current Heading: {self.heading}")

        # Calculate the error in heading
        heading_error = self.normalize_angle(desired_heading - self.heading)
        self.get_logger().info(f"Heading error: {heading_error}")

        #If we are heading towards the first waypoint, drive straight towards.
        if self.current_waypoint_idx == 0:
            self.get_logger().info(f"Driving straight to waypoint...")
            cmd = Twist()
            cmd.linear.x = float(1)  # Forward speed (m/s), adjust as needed
            cmd.angular.z = float(-0.5 * (heading_error))  # Turn speed (rad/s), adjust as needed
            
            self.auto_cmd_publisher.publish(cmd)

        #In this case we are already at a waypoint, so we need to ensure that we leave the cone to the robots right hand side. 
        else:
            if self.leaving_waypoint:
                self.get_logger().info(f"Turning around cone...")
                cmd = Twist()
                cmd.linear.x = float(1)
                cmd.angular.z = float(-0.3) #Turn right

                if abs(heading_error) < 0.5:
                    # We are facing the waypoint now, so follow earlier logic.
                    cmd.angular.z = float(0)
                    cmd.linear.x = float(1)
                    self.leaving_waypoint = False

                self.auto_cmd_publisher.publish(cmd)

            else:
                self.get_logger().info(f"Driving straight to waypoint...")
                cmd = Twist()
                cmd.linear.x = float(1)  # Forward speed (m/s), adjust as needed
                cmd.angular.z = float(-0.5 * (heading_error))  # Turn speed (rad/s), adjust as needed
                
                self.auto_cmd_publisher.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node = autoDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

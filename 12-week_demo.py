# import libraries
import roslibpy
import threading
from time import sleep

class RobotController:
    def __init__(self):
        # Connect to ROS network
        self.ip = '192.168.8.104'
        self.ros = roslibpy.Ros(host=self.ip, port=9012)
        self.ros.run()

        print(f"Connection: {self.ros.is_connected()}")

        # Define robot
        self.robot_name = 'bravo'

        # Define driving mode for robot
        self.drive_mode = "manual"

        # Create topic for wheel velocity
        self.vel_topic_name = f'/{self.robot_name}/cmd_vel'
        self.vel_topic = roslibpy.Topic(self.ros, self.vel_topic_name, 'geometry_msgs/Twist')

        # Create topic for IR sensor
        self.ir_topic_name = f'/{self.robot_name}/ir_intensity'
        self.ir_topic = roslibpy.Topic(self.ros, self.ir_topic_name, 'irobot_create_msgs/IrIntensityVector')

        # Create topic for receiving messages from JavaScript
        self.js_topic_name = '/web_to_python'
        self.js_topic = roslibpy.Topic(self.ros, self.js_topic_name, 'std_msgs/String')

        # Subscribe to topics
        self.ir_topic.subscribe(self.ir_callback)
        self.js_topic.subscribe(self.js_callback)

        # Set desired reflection of left and right sides
        self.left_act_reflect = 0
        self.right_act_reflect = 0
        self.front_act_reflect = 0
        self.left_des_reflect = 100
        self.right_des_reflect = 100

        self.kp = 0.005  # PD controller proportional gain
        self.kd = 0.002  # PD controller derivative gain
        self.prev_error = 0.0

        self.wall_mode = "center"

        # Wait for IR sensor data before starting
        while self.left_act_reflect == 0 and self.right_act_reflect == 0:
            print("Waiting for IR sensor data...")
            sleep(0.1)

        # Start drive loop in a separate thread
        threading.Thread(target=self.drive_loop, daemon=True).start()

    def ir_callback(self, message):
        """Receives IR sensor data and updates sensor values."""
        self.left_act_reflect = message['readings'][0]['value']
        self.right_act_reflect = message['readings'][6]['value']
        self.front_act_reflect = message['readings'][3]['value']

    def js_callback(self, message):
        """Handles messages from the JavaScript frontend."""
        print(f"Received message from JavaScript: {message['data']}")

        # Validate message format
        if not isinstance(message['data'], list) or len(message['data']) < 2:
            print("Invalid message format from JavaScript:", message)
            return

        new_drive_mode = message['data'][0]
        new_robot_name = message['data'][1]

        # Check if the robot name has changed
        if new_robot_name != self.robot_name:
            print(f"Switching to new robot: {new_robot_name}")

            # Unsubscribe from old IR topic
            self.ir_topic.unsubscribe(self.ir_callback)

            # Update robot name and recreate topics
            self.robot_name = new_robot_name

            self.vel_topic_name = f'/{self.robot_name}/cmd_vel'
            self.vel_topic = roslibpy.Topic(self.ros, self.vel_topic_name, 'geometry_msgs/Twist')

            self.ir_topic_name = f'/{self.robot_name}/ir_intensity'
            self.ir_topic = roslibpy.Topic(self.ros, self.ir_topic_name, 'irobot_create_msgs/IrIntensityVector')

            # Subscribe to new IR topic
            self.ir_topic.subscribe(self.ir_callback)

        # Update driving mode
        self.drive_mode = new_drive_mode

    def drive_loop(self):
        """Main driving loop using a PD controller with obstacle avoidance."""
        while True:
            if not self.ros.is_connected:
                print("ROS connection lost! Attempting to reconnect...")
                self.ros.run()

            if self.drive_mode == "manual":
                sleep(0.1)
                continue

            # PD controller variables
            z_angular = 0.0  
            current_error = 0.0  
            linear_speed = 0.2  

            # Determine wall mode based on sensor data
            if self.left_act_reflect == 0:
                self.wall_mode = "right"
            elif self.right_act_reflect == 0:
                self.wall_mode = "left"

            # Compute error for PD control
            if self.wall_mode == "right":
                print("Right Wall Following!")
                current_error = self.right_act_reflect - self.right_des_reflect
            elif self.wall_mode == "left":
                print("Left Wall Following!")
                current_error = self.left_des_reflect - self.left_act_reflect
            elif self.wall_mode == "center":
                print("Staying in the Center!")
                current_error = self.right_act_reflect - self.left_act_reflect

            # Compute derivative term
            derivative = (current_error - self.prev_error) / 0.1  

            # PD control equation
            z_angular = (self.kp * current_error) + (self.kd * derivative)

            # Obstacle avoidance
            if self.front_act_reflect > 200:
                print("Obstacle detected! Slowing down.")
                linear_speed = 0.05  
            elif self.front_act_reflect > 100:
                print("Object nearby, reducing speed.")
                linear_speed = 0.1  

            # Publish movement command
            drive = {"linear": {"x": linear_speed}, "angular": {"z": z_angular}}
            self.vel_topic.publish(roslibpy.Message(drive))

            self.prev_error = current_error
            sleep(0.1)

if __name__ == "__main__":
    robot = RobotController()

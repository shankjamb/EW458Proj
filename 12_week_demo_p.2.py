import threading
import roslibpy
from time import sleep

class RobotController:
    def __init__(self):

        self.thread_targets = [self.light_loop, self.drive_loop]
        self.threads = [threading.Thread(target=t, daemon=True) for t in self.thread_targets]

        # Connect to ROS network
        self.ip = '192.168.8.104'
        self.ros = roslibpy.Ros(host=self.ip, port=9012)
        self.ros.run()

        print(f"Connection: {self.ros.is_connected}")

        # Define robot
        self.robot_name = 'echo'

        # Define driving mode for robot
        self.drive_mode = "manual"

        # create topic for light ring
        self.light_topic_name = f'/{self.robot_name}/cmd_lightring'
        self.light_topic_type = 'irobot_create_msgs/LightringLeds'
        self.light_topic = roslibpy.Topic(self.ros, self.light_topic_name, self.light_topic_type)

        # Create topic for wheel velocity
        self.vel_topic_name = f'/{self.robot_name}/cmd_vel'
        self.vel_topic = roslibpy.Topic(self.ros, self.vel_topic_name, 'geometry_msgs/Twist')

        # Create topic for IR sensor
        self.ir_topic_name = f'/{self.robot_name}/ir_intensity'
        self.ir_topic = roslibpy.Topic(self.ros, self.ir_topic_name, 'irobot_create_msgs/IrIntensityVector')

        # Create topic for receiving messages from JavaScript
        self.js_topic_name = f'/{self.robot_name}/web_to_python'
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
        self.mode_lock_counter = 0  # Prevents rapid switching

        # Dictionaries for different colors
        self.led_yellow = [{'red': 255, 'green': 255, 'blue': 0} for _ in range(6)]
        self.led_blue = [{'red': 0, 'green': 0, 'blue': 255} for _ in range(6)]
        self.led_off = [{'red': 0, 'green': 0, 'blue': 0} for _ in range(6)]

        # Wait for IR sensor data before proceeding
        timeout = 5  # Max wait time (seconds)
        elapsed = 0
        while self.left_act_reflect == 0 and self.right_act_reflect == 0 and elapsed < timeout:
            print("Waiting for IR sensor data...")
            sleep(0.1)
            elapsed += 0.1

        if elapsed >= timeout:
            print("IR sensor data not received. Proceeding anyway...")


    def ir_callback(self, message):
        """Receives IR sensor data and updates sensor values."""
        self.left_act_reflect = message['readings'][0]['value']
        self.right_act_reflect = message['readings'][6]['value']
        self.front_act_reflect = message['readings'][3]['value']

    def js_callback(self, message):
        """Handles messages from the JavaScript frontend."""
        print(f"Received message from JavaScript: {message['data']}")

        if isinstance(message['data'], str):
            data_list = message['data'].split(",")
        else:
            return

        if len(data_list) < 2:
            return

        self.drive_mode = data_list[0]
        self.robot_name = data_list[1]

        print(f"Updated drive mode: {self.drive_mode}, Robot name: {self.robot_name}")

    def light_loop(self):
        while True:
            try:
                if self.drive_mode == 'manual':
                    print("Manual Light!")
                    msg_data = [{'override_system': True, 'leds': self.led_yellow},
                                {'override_system': True, 'leds': self.led_off}]
                elif self.drive_mode == 'autonomous':
                    print("Autonomous Light!")
                    msg_data = [{'override_system': True, 'leds': self.led_blue},
                                {'override_system': True, 'leds': self.led_off}]

                for i in msg_data:
                    msg = roslibpy.Message(i)
                    self.light_topic.publish(msg)
                    sleep(0.1)
            except:
                print('Problem with light loop!**************************')


    def drive_loop(self):
        #"""Main driving loop using a PD controller with obstacle avoidance."""
        
        linear_speed = 0.4  # Default speed
        last_known_error = 0.0  # Store last valid error when walls are missing
        
        while True:
            if self.drive_mode == "manual":
                sleep(0.1)
                continue

            elif self.drive_mode == "autonomous":
                z_angular = 0.0  # Initialize steering correction
                current_error = 0.0  # Reset error each loop

                # Wall detection
                left_detected = self.left_act_reflect > 0
                right_detected = self.right_act_reflect > 0

                # Compute error for PD control
                if left_detected and right_detected:
                    # Stay centered between walls
                    print("Center Tracking Mode")
                    current_error = self.right_act_reflect - self.left_act_reflect
                elif left_detected:
                    # Follow left wall
                    print("Left Wall Following")
                    current_error = self.left_des_reflect - self.left_act_reflect
                elif right_detected:
                    # Follow right wall
                    print("Right Wall Following")
                    current_error = self.right_act_reflect - self.right_des_reflect
                else:
                    # No walls detected, maintain last known error
                    print("No walls detected, maintaining last correction")
                    current_error = last_known_error

                # Save last known error if valid
                if left_detected or right_detected:
                    last_known_error = current_error

                # Compute derivative term for PD controller
                derivative = (current_error - self.prev_error) / 0.1  
                
                # PD control equation
                z_angular = (self.kp * current_error) + (self.kd * derivative)

                # Obstacle avoidance with maneuvering
                if self.front_act_reflect > 200:
                    print("Obstacle detected! Steering around it.")
                    linear_speed = 0.05
                    
                    if self.left_act_reflect < self.right_act_reflect:
                        z_angular = 0.5  # Turn left
                    else:
                        z_angular = -0.5  # Turn right
                elif self.front_act_reflect > 100:
                    print("Object nearby, reducing speed and preparing to turn.")
                    linear_speed = 0.1  
                    
                    if self.left_act_reflect < self.right_act_reflect:
                        z_angular = 0.3  # Slight left turn
                    else:
                        z_angular = -0.3  # Slight right turn
                else:
                    linear_speed = 0.4

                # Publish movement command
                drive = {"linear": {"x": linear_speed}, "angular": {"z": z_angular}}
                self.vel_topic.publish(roslibpy.Message(drive))

                self.prev_error = current_error
                sleep(0.1)  # Keep loop running in autonomous mode

                

    

    def start_threads(self):
        print(f'Starting {len(self.threads)} threads!')
        for t in self.threads:
            t.start()

if __name__ == "__main__":
    robot = RobotController()
    robot.start_threads()
    while 1:
        sleep(0.1)
        

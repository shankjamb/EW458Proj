import threading
import roslibpy
from time import sleep

class RobotController:
    def __init__(self):
        # Define the threads that will run in parallel
        self.thread_targets = [self.light_loop, self.drive_loop]
        self.threads = [threading.Thread(target=t, daemon=True) for t in self.thread_targets]

        # Connect to the ROS bridge server
        self.ip = '192.168.8.104'
        self.ros = roslibpy.Ros(host=self.ip, port=9012)
        self.ros.run()
        print(f"Connection: {self.ros.is_connected}")

        # Define the robot's name and initial drive mode
        self.robot_name = 'omega'
        self.drive_mode = "manual"

        # Topics for publishing light ring colors
        self.light_topic_name = f'/{self.robot_name}/cmd_lightring'
        self.light_topic_type = 'irobot_create_msgs/LightringLeds'
        self.light_topic = roslibpy.Topic(self.ros, self.light_topic_name, self.light_topic_type)

        # Topic for sending wheel velocity commands
        self.vel_topic_name = f'/{self.robot_name}/cmd_vel'
        self.vel_topic = roslibpy.Topic(self.ros, self.vel_topic_name, 'geometry_msgs/Twist')

        # Topic to receive IR sensor data
        self.ir_topic_name = f'/{self.robot_name}/ir_intensity'
        self.ir_topic = roslibpy.Topic(self.ros, self.ir_topic_name, 'irobot_create_msgs/IrIntensityVector')

        # Topic to receive messages from JavaScript (web interface)
        self.js_topic_name = f'/{self.robot_name}/web_to_python'
        self.js_topic = roslibpy.Topic(self.ros, self.js_topic_name, 'std_msgs/String')

        # Subscribe to IR and JavaScript topics
        self.ir_topic.subscribe(self.ir_callback)
        self.js_topic.subscribe(self.js_callback)

        # Initialize sensor values
        self.left_act_reflect = 0
        self.left_act_reflect_2 = 0
        self.front_left_act_reflect = 0
        self.front_act_reflect = 0
        self.front_right_act_reflect = 0
        self.right_act_reflect_2 = 0
        self.right_act_reflect = 0

        # Desired reflection values (targets for wall-following)
        self.left_des_reflect = 100
        self.right_des_reflect = 100

        # PD controller parameters
        self.kp = 0.005
        self.kd = 0.005
        self.prev_error = 0.0

        # Movement parameters
        self.z_angular = 0.0
        self.current_error = 0.0
        self.linear_speed = 0.4

        # Wall following mode (left, right, center)
        self.wall_mode = "center"

        # Light ring color definitions
        self.led_yellow = [{'red': 255, 'green': 255, 'blue': 0} for _ in range(6)]
        self.led_blue = [{'red': 0, 'green': 0, 'blue': 255} for _ in range(6)]
        self.led_off = [{'red': 0, 'green': 0, 'blue': 0} for _ in range(6)]

        # Wait for sensor data to initialize
        timeout = 5
        elapsed = 0
        while self.left_act_reflect == 0 and self.right_act_reflect == 0 and elapsed < timeout:
            print("Waiting for IR sensor data...")
            sleep(0.1)
            elapsed += 0.1
        if elapsed >= timeout:
            print("IR sensor data not received. Proceeding anyway...")

    def ir_callback(self, message):
        """Callback for updating all IR sensor values from message data."""
        self.left_act_reflect = message['readings'][0]['value']
        self.left_act_reflect_2 = message['readings'][1]['value']
        self.front_left_act_reflect = message['readings'][2]['value']
        self.front_act_reflect = message['readings'][3]['value']
        self.front_right_act_reflect = message['readings'][4]['value']
        self.right_act_reflect_2 = message['readings'][5]['value']
        self.right_act_reflect = message['readings'][6]['value']

    def js_callback(self, message):
        """Callback to update drive mode and robot name based on JS frontend input."""
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
        """Loop to update the light ring based on drive mode."""
        while True:
            try:
                if self.drive_mode == 'manual':
                    msg_data = [{'override_system': True, 'leds': self.led_yellow}, {'override_system': True, 'leds': self.led_off}]
                elif self.drive_mode == 'autonomous':
                    msg_data = [{'override_system': True, 'leds': self.led_blue}, {'override_system': True, 'leds': self.led_off}]
                for i in msg_data:
                    msg = roslibpy.Message(i)
                    self.light_topic.publish(msg)
                    sleep(0.1)
            except:
                print('Problem with light loop!**************************')

    def update_wall_mode(self):
        """Determines which wall-following mode to use based on IR sensors."""
        wall_threshold = 80
        left_avg = (self.left_act_reflect + self.left_act_reflect_2) / 2
        right_avg = (self.right_act_reflect + self.right_act_reflect_2) / 2
        front_vals = [self.front_act_reflect, self.front_left_act_reflect, self.front_right_act_reflect]
        front_avg = sum(front_vals) / len(front_vals)

        # Check if walls are detected in each direction
        left_wall = left_avg > wall_threshold
        right_wall = right_avg > wall_threshold
        front_wall = front_avg > wall_threshold

        # Decide wall mode
        if left_wall and right_wall:
            self.wall_mode = "center"
        elif right_wall:
            self.wall_mode = "right"
        elif left_wall:
            self.wall_mode = "left"
        elif front_wall:
            self.wall_mode = "right"  # fallback behavior
        else:
            self.wall_mode = "center"

    def drive_loop(self):
        """Main control loop that drives the robot autonomously using PD control."""
        while True:
            if self.drive_mode == "manual":
                sleep(0.1)
                continue

            elif self.drive_mode == "autonomous":
                self.update_wall_mode()
                print(f"Wall Mode: {self.wall_mode}")

                # Calculate error based on the current wall-following mode
                if self.wall_mode == "right":
                    print("Right Wall Following! 👉")
                    self.current_error = self.right_act_reflect - self.right_des_reflect
                elif self.wall_mode == "left":
                    print("Left Wall Following! 👈")
                    self.current_error = self.left_des_reflect - self.left_act_reflect
                elif self.wall_mode == "center":
                    print("Staying in the Center! 🫵")
                    self.current_error = self.right_act_reflect - self.left_act_reflect

                # PD controller
                derivative = (self.current_error - self.prev_error) / 0.1
                self.z_angular = (self.kp * self.current_error) + (self.kd * derivative)

                # Obstacle avoidance: adjust speed and turning near walls
                max_ir = max(self.front_act_reflect, self.front_left_act_reflect, self.front_right_act_reflect)
                safe_distance = 80
                danger_distance = 200

                if max_ir > safe_distance:
                    factor = min(1.0, (max_ir - safe_distance) / (danger_distance - safe_distance))
                    self.linear_speed = 0.4 - factor * 0.25
                    print(f"Obstacle detected! Slowing down. IR={max_ir}, Speed={self.linear_speed:.2f}")
                    self.z_angular += 0.2 * factor
                else:
                    self.linear_speed = 0.4

                # Send velocity command
                print(f"Linear speed: {self.linear_speed}, angular speed: {self.z_angular}")
                drive = {"linear": {"x": self.linear_speed}, "angular": {"z": self.z_angular}}
                self.vel_topic.publish(roslibpy.Message(drive))

                self.prev_error = self.current_error
                sleep(0.1)

    def start_threads(self):
        """Starts all robot control threads."""
        print(f'Starting {len(self.threads)} threads!')
        for t in self.threads:
            t.start()

# Entry point to start the robot controller
if __name__ == "__main__":
    robot = RobotController()
    robot.start_threads()
    while 1:
        sleep(0.1)

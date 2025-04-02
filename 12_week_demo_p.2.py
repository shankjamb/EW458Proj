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
        self.robot_name = 'bravo'

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

        self.wall_mode = "center" #default driving mode

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
        self.front_left_act_reflect = message['readings'][2]['value']
        self.front_right_act_reflect = message['readings'][4]['value']
        #print(self.front_act_reflect, self.front_left_act_reflect, self.front_right_act_reflect)


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
        """Main driving loop using a PD controller with obstacle avoidance."""
    
        while True:
            if self.drive_mode == "manual":
                sleep(0.1)
                continue

            elif self.drive_mode == "autonomous":
                # PD controller variables
                z_angular = 0.0  
                current_error = 0.0  
                linear_speed = 0.4

                #determine wall mode based on if see walls or not
                if self.right_act_reflect and self.left_act_reflect:
                    self.wall_mode = "center"
                elif  self.right_act_reflect:
                    self.wall_mode = "right"
                elif self.left_act_reflect:
                    self.wall_mode = "left"
                elif self.right_act_reflect and self.left_act_reflect==0:
                    self.wall_mode = "center"

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
                if (self.front_act_reflect > 300 or self.front_right_act_reflect > 300 or self.front_left_act_reflect > 300):
                    print("Obstacle detected! Slowing down.")
                    linear_speed = 0.05
                    z_angular = 0.4  
                # elif self.front_act_reflect or self.front_right_act_reflect or self.front_left_act_reflect> 150:
                #     print("Object nearby, reducing speed.")
                #     linear_speed = 0.1  
                #     z_angular = 0.4
                else:
                    linear_speed = 0.4

                # Publish movement command
                print(f"Linear speed: {linear_speed}, angular speed: {z_angular}")
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
        

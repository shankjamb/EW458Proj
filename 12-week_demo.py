# import libraries
import roslibpy
from time import sleep

class RobotController:
    def __init__(self):
        # Connect to ROS network
        self.ip = '192.168.8.104'
        self.ros = roslibpy.Ros(host=self.ip, port=9012)
        self.ros.run()

        print(f"Connection: {self.ros.is_connected}")

        # Define robot
        self.robot_name = 'bravo'

        # define driving mode for robot
        self.drive_mode = "manual"

        # Create topic for wheel velocity
        self.vel_topic_name = f'/{self.robot_name}/cmd_vel'
        self.vel_topic_type = 'geometry_msgs/Twist'
        self.vel_topic = roslibpy.Topic(self.ros, self.vel_topic_name, self.vel_topic_type)

        # Create topic for IR
        self.ir_topic_name = f'/{self.robot_name}/ir_intensity'
        self.ir_topic_type = 'irobot_create_msgs/IrIntensityVector'
        self.ir_topic = roslibpy.Topic(self.ros, self.ir_topic_name, self.ir_topic_type)

        # Create topic for receiving messages from JavaScript
        self.js_topic_name = '/web_to_python'
        self.js_topic_type = 'std_msgs/String'
        self.js_topic = roslibpy.Topic(self.ros, self.js_topic_name, self.js_topic_type)

        #set desired reflection of left and right side and initialize actual reflection of ir sensors
        self.left_act_reflect = 0
        self.right_act_reflect = 0
        self.left_des_reflect = 100
        self.right_des_reflect = 100
        self.kp = 0.005 #set gain for controller
        self.wall_mode = "center"  # Default mode

        self.prev_error = 0.0  # Store the previous error
        self.kd = 0.002        # Derivative gain, adjust as needed

        # Subscribe to IR topic
        self.ir_topic.subscribe(self.ir_callback)

    #set up method for ir sensor values 
    def ir_callback(self, message):
        """Receives callback and stores sensor values."""
        self.left_act_reflect = message['readings'][0]['value']
        self.right_act_reflect = message['readings'][6]['value']

    # Callback for receiving messages from JavaScript
    def js_callback(self, message):
        """Handles messages from the JavaScript frontend."""
        print(f"Received message from JavaScript: {message['data']}")
        self.drive_mode = message['data'][0]
        self.robot_name = message['data'][1]
        #need to get autonomous or manual mode from 
        #need to also get robot name from here


    #set up main drive method
    def drive_loop(self):
        """Main driving loop using a PD controller."""
        while True:
            
            if self.drive_mode == "manual":
                pass

            elif self.drive_mode == "autonomous":
                z_angular = 0.0  # Initialize angular velocity
                current_error = 0.0  # Initialize error variable

                if self.left_act_reflect == 0:
                    self.wall_mode="right"
                
                elif self.right_act_reflect == 0:
                    self.wall_mode="left"

                # Compute error for each mode
                if self.wall_mode == "right":
                    print("Right Wall Following!")
                    current_error = self.right_act_reflect - self.right_des_reflect

                elif self.wall_mode == "left":
                    print("Left Wall Following!")
                    current_error = self.left_des_reflect - self.left_act_reflect

                elif self.wall_mode == "center":
                    print("Staying in the Center!")
                    current_error = self.right_act_reflect - self.left_act_reflect

                # Compute derivative term (rate of error change)
                derivative = (current_error - self.prev_error) / 0.1  # Assuming 0.1s loop time

                # PD control equation
                z_angular = (self.kp * current_error) + (self.kd * derivative)

                print(f"Mode: {self.wall_mode}, Error: {current_error}, Derivative: {derivative}, z_angular: {z_angular}")

                # Send movement command
                drive = {"linear": {"x": 0.2, "y": 0.0, "z": 0.0}, 
                        "angular": {"x": 0.0, "y": 0.0, "z": z_angular}}

                msg = roslibpy.Message(drive)
                self.vel_topic.publish(msg)

                # Store current error for next iteration
                self.prev_error = current_error

                sleep(0.1)  # Adjust loop timing


if __name__ == "__main__":

    robot = RobotController()
    robot.drive_loop()

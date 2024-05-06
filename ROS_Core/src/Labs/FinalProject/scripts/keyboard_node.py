import rospy
# !!IMPORTANT: You need to install the pynput module to use this script
# pip install pynput
from pynput import keyboard
from racecar_msgs.msg import ServoMsg 
import threading
from geometry_msgs.msg import PoseStamped

class KeyboardControlNode:
    """
    A ROS node that allows keyboard control of a racecar using upper, down, left and right keys.

    This node listens for keyboard inputs and publishes velocity commands to control the racecar's throttle and steering.

    Attributes:
        max_throttle (float): The maximum throttle value.
        decay_factor (float): The decay factor for reducing throttle.
        accel_rate (float): The rate at which the throttle accelerates.
        steering_rate (float): The rate at which the steering angle changes.
        control_topic (str): The topic to publish the control commands.
        control_pub (rospy.Publisher): The publisher for the control commands.
        throttle (float): The current throttle value.
        steering (float): The current steering angle.
        rate (rospy.Rate): The rate at which the node runs.
        up (bool): Flag indicating if the up arrow key is pressed.
        down (bool): Flag indicating if the down arrow key is pressed.
        left (bool): Flag indicating if the left arrow key is pressed.
        right (bool): Flag indicating if the right arrow key is pressed.
    """

    def __init__(self):
        """
        Initializes the KeyboardControlNode.
        """
        # Get parameters from the ROS parameter server
        # TODO: You may want to change the default values of the parameters so that the car speed up and slow down as your desired rate
        self.max_throttle = rospy.get_param('~max_throttle', 0.2)
        self.decay_factor = rospy.get_param('~decay_factor', 0.9)
        self.accel_rate = rospy.get_param('~accel_rate', 0.05)
        self.steering_rate = rospy.get_param('~steering_rate', 0.5)
        self.control_topic = rospy.get_param('~control_topic', '/Control')
        # Create a publisher for the robot's velocity commands
        self.control_pub = rospy.Publisher("/keyboard_control", ServoMsg, queue_size=10)
                
        self.throttle = -5
        self.steering = 0.0
        self.rate = rospy.Rate(50)
        
        # flags for the keyboard
        self.up = False
        self.down = False
        self.left = False
        self.right = False
        
        threading.Thread(target=self.start_listen).start()
        threading.Thread(target=self.run).start()

    def start_listen(self):
        """
        Starts listening for keyboard inputs.
        """
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        
        
    def on_press(self, key):
        """
        Callback function for when a key is pressed.

        Args:
            key (keyboard.Key): The key that was pressed.
        """
        print("press")
        if key == keyboard.Key.up:
            self.up = True
        elif key == keyboard.Key.down:
            self.down = True
        elif key == keyboard.Key.left:
            self.left = True
        elif key == keyboard.Key.right:
            self.right = True
        
    def on_release(self, key):
        """
        Callback function for when a key is released.

        Args:
            key (keyboard.Key): The key that was released.
        """
        if key == keyboard.Key.up:
            self.up = False
        elif key == keyboard.Key.down:
            self.down = False
        elif key == keyboard.Key.left:
            self.left = False
        elif key == keyboard.Key.right:
            self.right = False
        
    def run(self):
        """
        Main loop of the node.
        """
        while not rospy.is_shutdown():    
            servo_msg = ServoMsg()
            # Set the header time to the current time
            servo_msg.header.stamp = rospy.Time.now()
            # Set the throttle and steering angle
            
            if self.up:
                self.throttle = min(self.throttle + self.accel_rate, self.max_throttle)
            elif self.down:
                self.throttle = max(self.throttle - self.accel_rate, -5)
            else:
                self.throttle = max(self.throttle * self.decay_factor, 0)
            
            if self.left:
                if self.steering > 0:
                    self.steering = 0
                self.steering = min(self.steering + self.steering_rate, 1)
            elif self.right:
                if self.steering < 0:
                    self.steering = 0
                self.steering = max(self.steering - self.steering_rate, -1)
            else:
                self.steering = 0
                
            servo_msg.throttle = self.throttle
            servo_msg.steer = self.steering
            self.control_pub.publish(servo_msg)
            self.rate.sleep()


if __name__ == '__main__':

    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('keyboard_publisher', anonymous=True)

    print("Starting Keyboard Control Node...")
    node = KeyboardControlNode()
# rospy.init_node('keyboard_control')
# pub = rospy.Publisher('/keyboard_control', String, queue_size=10)
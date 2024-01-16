#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy


class CubePosition:
    def __init__(self):
        self.position_pub = rospy.Publisher('/cube_position', Float64MultiArray, queue_size=1) # define a publisher tosend position data from python to unity
        self.key_sub = rospy.Subscriber('/keyboard_command', Float64MultiArray, self.keyboard_callback) # define a subscriber to receive keyboard data from unity to python (if data comes will automatically call keyboard_callback function)
        self.joy_sub = rospy.Subscriber('/joy_command', Joy, self.joy_callback) # define a subscriber to receive joy data from unity to python (if data comes will automatically call joy_callback function)
        self.position_msg = Float64MultiArray()
        self.position_msg.data = [0, 0]

    def joy_callback(self, msg):
        #print(f"joy_input z:{msg.axes[0]},x:{msg.axes[1]}")
        z_axis = msg.axes[0] # this is about unity axis (forward is 1, backward is -1) 
        x_axis = msg.axes[1] # this is about unity axis (right is 1, left is -1)
        self.position_msg.data[0] += z_axis  
        self.position_msg.data[1] += x_axis 
        self.position_pub.publish(self.position_msg)

        # button status check
        #now it is unused but you can use it if you want (trigger is 1, untrigger is 0)
        primary_button_trigger = msg.buttons[0]
        secondary_button_trigger = msg.buttons[1]
        grip_button_trigger = msg.buttons[2]
        trigger_button_trigger = msg.buttons[3]
        #print(f"button status: {primary_button_trigger}, {secondary_button_trigger}, {grip_button_trigger}, {trigger_button_trigger}")


    def keyboard_callback(self, msg):
        #print(f"kyboard_input z:{msg.data[0]},x:{msg.data[1]}")
        z_axis = msg.data[0]
        x_axis = msg.data[1]
        self.position_msg.data[0] += z_axis  
        self.position_msg.data[1] += x_axis 
        self.position_pub.publish(self.position_msg)
    
if __name__ == "__main__":
    rospy.init_node('cube_position_node')
    cube_position = CubePosition()
    while not rospy.is_shutdown():
        print(f"cube position:{cube_position.position_msg.data} ", end='\r')


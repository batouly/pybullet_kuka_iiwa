#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point  # For sending Cartesian positions of the end effector
import time
from std_msgs.msg import Bool

class KukaClientNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('kuka_client_node')
        # Node parameters
        self.target_position_topic = rospy.get_param('/target_position_topic', '/target_position')
        self.gripper_state_topic = rospy.get_param('/gripper_state_topic', '/gripper_state')
        self.cube_pose_topic = rospy.get_param('/cube_pose_topic', '/cube_pose')
        self.pick_toggle_topic = rospy.get_param('/pick_toggle_topic', '/pick_toggle')
        # Set a loop rate
        self.rate = rospy.Rate(10)  # 10 Hz
        self.cube_pose = []
        self.waypoints = [] # format [target_pose, gripper_state]
        self.pick_cmd_flag = False
        self.initial = [0.8, -0.2, 1.1]
        self.target = [0.85, 0, 0.95]

        #Publisher for sending end-effector position commands to the server
        self.pub = rospy.Publisher(self.target_position_topic, Point, queue_size=10)
        #Publisher for sending gripper_state to the server
        self.pub1 = rospy.Publisher(self.gripper_state_topic, Bool, queue_size=10)
        #Subscriber to cube position for picking
        rospy.Subscriber(self.cube_pose_topic, Point, self.cube_pose_callback)
        #Subscriber to state of toggle button in server
        rospy.Subscriber(self.pick_toggle_topic, Bool, self.pick_cmd_callback)

    def pick_cmd_callback(self,msg):
        self.pick_cmd_flag = msg.data

    def send_position_commands(self,i):
        # Create a Point message to send Cartesian positions
        position_msg = Point()
        # Set the desired position of the end effector
        position_msg.x, position_msg.y, position_msg.z = self.waypoints[i][0]
        # Publish the Cartesian position to the server
        self.pub.publish(position_msg)
        self.pub1.publish(self.waypoints[i][1])

    def cube_pose_callback(self,msg):
            self.cube_pose = [msg.x, msg.y, msg.z]
            
    def set_waypoints(self,x,y,z):
        # set waypoints according to cube pose
        self.waypoints = [
                            [[x, y, z + 0.5], 0],
                            [[x, y, z + 0.27], 0],
                            [[x, y, z + 0.27], 1], 
                            [[x, y, z + 0.3], 1], 
                            [self.target, 1],
                            [self.target, 0], 
                            [self.initial, 0]] 
        
    def run(self):
        # Main loop to send commands
        while not rospy.is_shutdown():
            # Send the current target position
            if self.pick_cmd_flag:
                
                self.set_waypoints(self.cube_pose[0],self.cube_pose[1],self.cube_pose[2])
                for i in range(len(self.waypoints)):
                    self.send_position_commands(i)
                    time.sleep(1)
                  
            # Sleep to have time to toggle button in GUI
            time.sleep(3)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        # Create the client node and start running
        client_node = KukaClientNode()
        client_node.run()

    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
import time
import math
import pybullet as p
import pybullet_data
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from pybullet_interfaces.srv import EndEffectorControl, EndEffectorControlResponse
from sensor_msgs.msg import JointState

class KukaServerNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('kuka_server_node')
        # Node parameters
        self.gripper_force = rospy.get_param('/gripper_force', 100)
        self.joint_state_topic = rospy.get_param('/joint_state_topic', '/joint_states')
        self.target_position_topic = rospy.get_param('/target_position_topic', '/target_position')
        self.gripper_state_topic = rospy.get_param('/gripper_state_topic', '/gripper_state')
        self.cube_pose_topic = rospy.get_param('/cube_pose_topic', '/cube_pose')
        self.pick_toggle_topic = rospy.get_param('/pick_toggle_topic', '/pick_toggle')
        self.ee_control_service_name = rospy.get_param('/ee_control_service_name', '/ee_control_service')

        # Subscribers
        rospy.Subscriber(self.target_position_topic, Point, self.target_position_callback)
        rospy.Subscriber(self.gripper_state_topic , Bool, self.gripper_state_callback)
        # Publisher for cube position
        self.cube_pose_pub = rospy.Publisher(self.cube_pose_topic, Point, queue_size=10)
        # Publisher of button state
        self.pick_cmd_pub = rospy.Publisher(self.pick_toggle_topic , Bool, queue_size=10)
        # Create the publisher to publish the joint states
        self.joint_state_pub = rospy.Publisher(self.joint_state_topic, JointState, queue_size=10)
        # ROS Service to receive target position and gripper state
        self.target_position_service = rospy.Service(self.ee_control_service_name, EndEffectorControl, self.target_position_service)


        # Set up PyBullet environment
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.button = p.addUserDebugParameter("  Toggle Pick/Stop",1,0,0) ## Add button to act as a toggle
        

        # Load environment and robot
        self.plane_id = p.loadURDF("plane.urdf")
        self.kuka_id = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000, -0.200000, 0.600000, 0.000000, 0.000000, 0.000000, 1.000000)
        self.kuka_gripper_id = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
        self.table_id = p.loadURDF("table/table.urdf", basePosition=[1.0, -0.2, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
        self.cube_id = p.loadURDF("cube.urdf", basePosition=[0.85, -0.2, 0.65], globalScaling=0.05)
        self.num_joints = p.getNumJoints(self.kuka_id)
        # Joint names for the KUKA iiwa (for publishing joint states)
        self.joint_names = [f"joint_{i}" for i in range(self.num_joints)]
        
        # Attach the gripper to the KUKA arm
        self.kuka_cid = p.createConstraint(self.kuka_id, 6, self.kuka_gripper_id, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.0], [0, 0, 0])
        self.kuka_cid2 = p.createConstraint(self.kuka_gripper_id, 4, self.kuka_gripper_id, 6, jointType=p.JOINT_GEAR, jointAxis=[1, 1, 1], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
        p.changeConstraint(self.kuka_cid2, gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=self.gripper_force)

        # Increase the friction between the gripper and the cube
        self.set_friction(self.cube_id, lateral_friction=10.0, spinning_friction=1.0, rolling_friction=1.0)

        # Initialize variables for target position and gripper state
        self.target_pos = [0.85, -0.2, 1.1]  # Default initial position
        self.gripper_val = 0  # Default gripper state (open)
        self.target_reached = True  # Flag to track if the target is reached
        self.previous_pose = [0.85, -0.2, 1.1]

        # Joint limits for the KUKA (these are just examples, adjust according to the URDF of your robot)
        self.lower_limits = [-0.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.054]
        self.upper_limits = [0.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.054]

        # Initialize KUKA and gripper joint states
        self.reset_kuka()
        self.reset_gripper()

        # ROS rate
        self.rate = rospy.Rate(100)  # 100 Hz loop rate
        
        # Run the main loop
        self.run()
        

    def publish_joint_states(self, joint_positions):
        # Create a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_positions  # Use the calculated joint positions

        # Publish the joint state message
        self.joint_state_pub.publish(joint_state_msg)

    def is_within_limits(self,joint_poses):
        """Check if the joint angles are within the joint limits"""
        for i in range(len(joint_poses)):
            if joint_poses[i] < self.lower_limits[i] or joint_poses[i] > self.upper_limits[i]:
                return False
        return True

    def reset_kuka(self):
        """ Reset KUKA joint positions"""
        joint_positions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
        for joint_index in range(p.getNumJoints(self.kuka_id)):
            p.resetJointState(self.kuka_id, joint_index, joint_positions[joint_index])

    def reset_gripper(self):
        """Reset gripper joint positions"""
        p.resetBasePositionAndOrientation(self.kuka_gripper_id, [0.923103, -0.200000, 1.250036], [-0.000000, 0.964531, -0.000002, -0.263970])
        joint_positions = [0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000]
        for joint_index in range(p.getNumJoints(self.kuka_gripper_id)):
            p.resetJointState(self.kuka_gripper_id, joint_index, joint_positions[joint_index])

    def set_friction(self, body_id, lateral_friction, spinning_friction, rolling_friction):
        """Set the friction properties of the gripper and cube"""
        p.changeDynamics(body_id, -1, lateralFriction=lateral_friction)
        p.changeDynamics(body_id, -1, spinningFriction=spinning_friction)
        p.changeDynamics(body_id, -1, rollingFriction=rolling_friction)

    def control_gripper(self):
        """Control the gripper and apply more realistic gripping force"""
        p.setJointMotorControl2(self.kuka_gripper_id, 4, p.POSITION_CONTROL, targetPosition=0.05 * self.gripper_val, force=2 * self.gripper_force)  # Increase force
        p.setJointMotorControl2(self.kuka_gripper_id, 6, p.POSITION_CONTROL, targetPosition=0.05 * self.gripper_val, force=2 * self.gripper_force)
        time.sleep(1)  # Allow the gripper to close and simulate a strong grip

    def target_position_callback(self, msg):
        """ Update target position from the ROS message"""
        if self.previous_pose != [msg.x, msg.y, msg.z]:
            self.target_pos = [msg.x, msg.y, msg.z]
            rospy.loginfo(f"Received new target position: {self.target_pos}")
            self.target_reached = False
        else:
            self.target_reached = True
        self.previous_pose = [msg.x, msg.y, msg.z]

    def target_position_service(self, req):
        """Handle service request to update the target position and gripper state"""
        self.gripper_val = req.gripper_state
        self.control_gripper()
        if self.previous_pose != [req.ee_position.x, req.ee_position.y, req.ee_position.z]:
            self.target_pos = [req.ee_position.x, req.ee_position.y, req.ee_position.z]
            rospy.loginfo(f"Received new target position: {self.target_pos}")
            self.target_reached = False
        else:
            self.target_reached = True
        self.previous_pose = self.target_pos 
        return EndEffectorControlResponse(success=True)

    def gripper_state_callback(self, msg):
        """Update gripper state (True for closed, False for open)"""
        self.gripper_val = 1 if msg.data else 0
        self.control_gripper()
        rospy.loginfo(f"Received new gripper state: {'Closed' if self.gripper_val else 'Open'}")

    def interpolate(self, start, end, fraction):
        """Interpolate between two points with a given fraction"""
        return [start[i] + fraction * (end[i] - start[i]) for i in range(len(start))]

    def move_to_target(self):
        """Move the robot to the target position and check if the target is reached"""
        interpolation_steps = 200
        step_in_interpolation = 0

        while step_in_interpolation < interpolation_steps:
            fraction = step_in_interpolation / float(interpolation_steps)
            current_pos = self.interpolate(self.previous_pose, self.target_pos, fraction)

            # Get target orientation (fixed for simplicity)
            target_orn = p.getQuaternionFromEuler([0, math.pi, 0])

            # Calculate inverse kinematics for the target position and orientation
            joint_poses = p.calculateInverseKinematics(self.kuka_id, 6, current_pos, target_orn)
            self.publish_joint_states(joint_poses) #JointState if needed by other clients
            if not self.is_within_limits(joint_poses):
                print(f"Position {current_pos} is out of the working space.")
                break
            # Set joint positions using the calculated inverse kinematics
            for j in range(p.getNumJoints(self.kuka_id)):
                p.setJointMotorControl2(bodyIndex=self.kuka_id, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=joint_poses[j])

            # Step the simulation
            p.stepSimulation()

            step_in_interpolation += 1
            # Sleep to maintain the loop rate
            self.rate.sleep()

        # Mark the target as reached
        self.target_reached = True
        rospy.loginfo(f"Target position {self.target_pos} reached.")

    def run(self):
        """Main loop for controlling the kuka"""
        while not rospy.is_shutdown():
            if not self.target_reached:
                # If the target has not been reached, move towards the target
                self.move_to_target()
    
            value = p.readUserDebugParameter(self.button) % 2 # True for odd and False for even
            self.pick_cmd_pub.publish(bool(value)) # Publish button state

            if not value:
                rospy.loginfo_once("Change the cube position with the mouse or click the button on the right!")
            else:
                rospy.loginfo_once("Start pick and place!")

            self.cubePos, _ = p.getBasePositionAndOrientation(self.cube_id)
            msg = Point()
            msg.x, msg.y, msg.z = self.cubePos
            self.cube_pose_pub.publish(msg) # Publish cube position

            # Step the simulation
            p.stepSimulation()

            # Sleep to maintain the loop rate
            self.rate.sleep()

        # Disconnect the simulation on shutdown
        p.disconnect()

if __name__ == '__main__':
    try:
        KukaServerNode()  # Initialize and run the node
    except rospy.ROSInterruptException:
        pass

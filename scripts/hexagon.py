#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from franka_msgs.msg import FrankaState
from tf.transformations import quaternion_from_euler
import time

class WaypointTrajectoryNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('hexagonal_trajectory_node')

        # Publisher to send Cartesian target poses
        self.pose_pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=10)

        # Duration to stay at each waypoint (seconds)
        self.waypoint_duration = rospy.get_param('~waypoint_duration', 3.0)  # 2 seconds at each waypoint

        # Rate to publish poses (optional if we want to control frequency)
        self.rate = rospy.Rate(100)  # 100 Hz publish rate

        # Waypoints in the form: [X, Y, Z, alpha, beta, gamma]
        self.waypoints = [
            [0.55, 0.15, 0.3, 0, 0, 0],
            [0.55, -0.15, 0.3, 0, 0, 0],
            [0.55, -0.3, 0.56, 0, 0.785375, 0],
            [0.55, -0.15, 0.82, 0, 1.04716666666667, 0],
            [0.55, 0.15, 0.82, 0, 1.04716666666667, 0],
            [0.55, 0.3, 0.56, 0, 0.785375, 0],
            [0.55, 0.15, 0.3, 0, 0, 0]
        ]

        # Get the initial pose from the robot (optional)
        self.initial_pose = self.get_initial_pose()

    def get_initial_pose(self):
        """
        Fetch the current pose from the Franka state topic.
        """
        rospy.loginfo("Waiting for Franka state topic...")
        msg = rospy.wait_for_message("/franka_state_controller/franka_states", FrankaState)
        initial_pose = PoseStamped()
        initial_pose.pose.position.x = msg.O_T_EE[12]
        initial_pose.pose.position.y = msg.O_T_EE[13]
        initial_pose.pose.position.z = msg.O_T_EE[14]
        # Assuming the orientation does not change
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 1.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 0.0  # Identity quaternion
        return initial_pose

    def send_waypoint(self, position, orientation):
        """
        Send a single waypoint to the robot.
        """
        target_pose = PoseStamped()
        target_pose.header = Header()
        target_pose.header.stamp = rospy.Time.now()

        # Set position
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2]

        # Set orientation (using quaternions converted from Euler angles)
        target_pose.pose.orientation.x = orientation[0]
        target_pose.pose.orientation.y = orientation[1]
        target_pose.pose.orientation.z = orientation[2]
        target_pose.pose.orientation.w = orientation[3]

        # Publish the target pose
        self.pose_pub.publish(target_pose)

    def run(self):
        """
        Main loop to send the waypoints sequentially.
        """
        rospy.loginfo("Starting waypoint trajectory...")

        for waypoint in self.waypoints:
            # Extract position and Euler angles from waypoint
            x, y, z, alpha, beta, gamma = waypoint

            # Convert Euler angles (alpha, beta, gamma) to quaternion
            quaternion = quaternion_from_euler(alpha, beta, gamma)

            # Send the current waypoint
            self.send_waypoint([x, y, z], quaternion)

            rospy.loginfo(f"Sent waypoint: x={x}, y={y}, z={z}, alpha={alpha}, beta={beta}, gamma={gamma}")

            # Wait at the waypoint for the specified duration
            time.sleep(self.waypoint_duration)

        rospy.loginfo("Waypoint trajectory completed.")

if __name__ == '__main__':
    try:
        node = WaypointTrajectoryNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
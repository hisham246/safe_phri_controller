#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from franka_msgs.msg import FrankaState

class CircularTrajectoryNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('circular_trajectory_node')

        # Publisher to send Cartesian target poses
        self.pose_pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=10)

        # Desired frequency and amplitude of the circular trajectory
        self.frequency = rospy.get_param('~frequency', 0.05)  # Hz (how fast the circle is traced)
        self.amplitude = rospy.get_param('~amplitude', 0.5)  # meters (radius of the circle)
        self.duration = rospy.get_param('~duration', 30)      # seconds

        # Rate at which to publish commands
        self.rate = rospy.Rate(100)  # 100 Hz control loop

        # Initial pose (can be obtained from FrankaState or set manually)
        self.initial_pose = self.get_initial_pose()

    def get_initial_pose(self):
        """
        Fetch the current pose from the robot state or define a default.
        """
        rospy.loginfo("Waiting for Franka state topic...")
        msg = rospy.wait_for_message("/franka_state_controller/franka_states", FrankaState)
        initial_pose = PoseStamped()
        initial_pose.pose.position.x = msg.O_T_EE[12]
        initial_pose.pose.position.y = msg.O_T_EE[13]
        initial_pose.pose.position.z = msg.O_T_EE[14]
        # Assuming the orientation does not change
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 0.0
        return initial_pose

    def run(self):
        """
        Main loop to generate and publish the circular trajectory.
        """
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec() - start_time
            if current_time > self.duration:
                rospy.loginfo("Trajectory complete.")
                break

            # Generate circular position offsets for x and y directions
            x_offset = self.amplitude * np.cos(2 * np.pi * self.frequency * current_time)
            y_offset = self.amplitude * np.sin(2 * np.pi * self.frequency * current_time)

            # Construct the new target pose
            target_pose = PoseStamped()
            target_pose.header = Header()
            target_pose.header.stamp = rospy.Time.now()

            # Apply the circular offset to the x and y coordinates
            target_pose.pose.position.x = self.initial_pose.pose.position.x + x_offset
            target_pose.pose.position.y = self.initial_pose.pose.position.y + y_offset
            target_pose.pose.position.z = self.initial_pose.pose.position.z  # Keep z constant

            # Maintain the same orientation (identity quaternion)
            target_pose.pose.orientation = self.initial_pose.pose.orientation

            # Publish the pose
            self.pose_pub.publish(target_pose)

            # Sleep to maintain control rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CircularTrajectoryNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
import pandas as pd
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

csv_file = '/home/robohub/demos/demo_2024-09-17-09-12-09.csv'
df = pd.read_csv(csv_file)


rospy.init_node('panda_trajectory_publisher', anonymous=True)
publisher = rospy.Publisher('/cartesian_impedance_controller/desired_pose', Pose, queue_size=10)
rate = rospy.Rate(100)  # 100 Hz

for index, row in df.iterrows():
    pose_msg = Pose()
    
    pose_msg.position.x = row[df.columns[0]]
    pose_msg.position.y = row[df.columns[1]]
    pose_msg.position.z = row[df.columns[2]]
    
    pose_msg.orientation.x = row[df.columns[3]]
    pose_msg.orientation.y = row[df.columns[4]]
    pose_msg.orientation.z = row[df.columns[5]]
    pose_msg.orientation.w = row[df.columns[6]]
    
    publisher.publish(pose_msg)
    
    rate.sleep()

rospy.spin()

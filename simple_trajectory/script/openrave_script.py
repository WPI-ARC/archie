#!/bin/python
joints_names = ["r_shoulder_pan_joint" , "r_shoulder_lift_joint" , "r_upper_arm_roll_joint" , "r_elbow_flex_joint" , "r_forearm_roll_joint" , "r_wrist_flex_joint" , "r_wrist_roll_joint" ]
robot = env.GetRobots()[0]
robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in joints_names])
print robot.GetActiveDOFValues()

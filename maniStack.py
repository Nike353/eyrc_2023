#! /usr/bin/env python3

import sys
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from sensor_msgs.msg import CameraInfo
import tf
from std_msgs.msg import String
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler

count=0
fruit = []
pub=rospy.Publisher("/mani_nav",String,queue_size=10)

def nav_mani_callback(data):
    global count
    global fruit 
    fruit = data.data
    

#initialize
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('arm_manipulation_perception',anonymous=True)
robot=moveit_commander.RobotCommander()
scene=moveit_commander.PlanningSceneInterface()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
execute_trajectory_client=actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
execute_trajectory_client.wait_for_server()
rospy.Subscriber("/nav_mani",String,nav_mani_callback)


# #arm rest
arm_group=moveit_commander.MoveGroupCommander("arm")
hand_group=moveit_commander.MoveGroupCommander("gripper")

# #left view
# lst_joint_angles_1 = [math.radians(85.5),
#                         math.radians(-53),
#                         math.radians(-1),
#                         math.radians(40),
#                         math.radians(1),
#                         math.radians(1)]
# arm_group.set_joint_value_targetlst_joint_angles_1)
# arm_group.plan()
# flag_plan_1=arm_group.go()

lst_joint_angles_1 = [math.radians(85.5),
                        math.radians(-5),
                        math.radians(-15),
                        math.radians(0),
                        math.radians(0),
                        math.radians(0)]
arm_group.set_joint_value_target(lst_joint_angles_1)
arm_group.plan()
flag_plan_1=arm_group.go()


rospy.sleep(1)

while not rospy.is_shutdown():
    if fruit=="PLUCK_RED":
    
        listener = tf.TransformListener()
        listener.waitForTransform("fruit_red", "ebot_base", rospy.Time(), rospy.Duration(4))
        (trans_r,rot_r) = listener.lookupTransform('ebot_base', 'fruit_red', rospy.Time(0))
        red_pose = geometry_msgs.msg.Pose()
        red_pose.position.x = trans_r[0]-0.01
        red_pose.position.y = trans_r[1]-0.20
        red_pose.position.z = trans_r[2]-0.01
        (red_pose.orientation.x,red_pose.orientation.y,red_pose.orientation.z,red_pose.orientation.w) = quaternion_from_euler(2.833,0.0009,3.108)
        pose_values = arm_group.get_current_pose().pose
        arm_group.set_pose_target(red_pose)
        flag_plan = arm_group.go()
        hand_group.set_named_target("close")
        plan1=hand_group.go()
        #revert back to red basket position
        lst_joint_angles_3 = [math.radians(-20),
                            math.radians(0),
                            math.radians(0),
                            math.radians(0),
                            math.radians(0),
                            math.radians(0)]
        arm_group.set_joint_value_target(lst_joint_angles_3)
        arm_group.plan()
        flag_plan_3=arm_group.go()

        #gripper open to drop
        hand_group.set_named_target("open")
        plan2=hand_group.go()
        
                #left view
        lst_joint_angles_1 = [math.radians(85.5),
                                math.radians(-53),
                                math.radians(-1),
                                math.radians(40),
                                math.radians(1),
                                math.radians(1)]
        arm_group.set_joint_value_target(lst_joint_angles_1)
        arm_group.plan()
        flag_plan_1=arm_group.go()

        
        arm_group.set_joint_value_target(lst_joint_angles_1)
        arm_group.plan()
        flag_plan_1=arm_group.go()
        pub.publish("OKIE")
        pub.publish("none")
        
    elif fruit=="PLUCK_YELLOW":
        
        listener = tf.TransformListener()
        listener.waitForTransform("fruit_yellow", "ebot_base", rospy.Time(), rospy.Duration(5))
        (trans_y,rot_y) = listener.lookupTransform('ebot_base', 'fruit_yellow', rospy.Time(0))    
        
        

        yellow_pose = geometry_msgs.msg.Pose()
        yellow_pose.position.x = trans_y[0]+0.05
        yellow_pose.position.y = trans_y[1]-0.21
        yellow_pose.position.z = trans_y[2]+0.18
        (yellow_pose.orientation.x,yellow_pose.orientation.y,yellow_pose.orientation.z,yellow_pose.orientation.w) = quaternion_from_euler(-2.604,-0.001,-2.7208)
        pose_values = arm_group.get_current_pose().pose
        arm_group.set_pose_target(yellow_pose)
        flag_plan = arm_group.go()

        hand_group.set_named_target("close")
        plan1=hand_group.go()

        lst_joint_angles_7 = [math.radians(15),
                     math.radians(0),
                     math.radians(0),
                     math.radians(0),
                     math.radians(0),
                     math.radians(0)]
        arm_group.set_joint_value_target(lst_joint_angles_7)
        arm_group.plan()
        flag_plan_7=arm_group.go()

        hand_group.set_named_target("open")
        plan4=hand_group.go()

        pub.publish("OKIE")
        arm_group.set_joint_value_target(lst_joint_angles_1)
        arm_group.plan()
        flag_plan_1=arm_group.go()
    
    else:
        pass

#rightview
# lst_joint_angles_1 = [math.radians(-90),
#                         math.radians(-45),
#                         math.radians(2),
#                         math.radians(29),
#                         math.radians(1),
#                         math.radians(1)]
# arm_group.set_joint_value_target(lst_joint_angles_1)
# arm_group.plan()
# flag_plan_1=arm_group.go()

# listener = tf.TransformListener()
# listener.waitForTransform("fruit_red", "ebot_base", rospy.Time(), rospy.Duration(4))
# (trans_r,rot_r) = listener.lookupTransform('ebot_base', 'fruit_red', rospy.Time(0))

# listener.waitForTransform("fruit_yellow", "ebot_base", rospy.Time(), rospy.Duration(4))
# (trans_y,rot_y) = listener.lookupTransform('ebot_base', 'fruit_yellow', rospy.Time(0))




# arm_group.set_named_target("rest")
# plan1=arm_group.go()

# rospy.sleep(1)

# #def movement():
# lst_joint_angles_1 = [math.radians(90),
#                      math.radians(5),
#                      math.radians(-66),
#                      math.radians(53),
#                      math.radians(0),
#                      math.radians(0)]
# arm_group.set_joint_value_target(lst_joint_angles_1)
# arm_group.plan()
# flag_plan_1=arm_group.go()


# lst_joint_angles_3 = [math.radians(-20),
#                      math.radians(0),
#                      math.radians(0),
#                      math.radians(0),
#                      math.radians(0),
#                      math.radians(0)]
# arm_group.set_joint_value_target(lst_joint_angles_3)
# arm_group.plan()
# flag_plan_3=arm_group.go()





#rightview
# lst_joint_angles_1 = [math.radians(-90),
#                         math.radians(-45),
#                         math.radians(2),
#                         math.radians(29),
#                         math.radians(1),
#                         math.radians(1)]
# arm_group.set_joint_value_target(lst_joint_angles_1)
# arm_group.plan()
# flag_plan_1=arm_group.go()
#main
# centre_value = rospy.Subscriber('/camera/color/camera_info2', CameraInfo, callback=movement)

############################# RED PEPPER #########################################



# #arm go to red pepper
# lst_joint_angles_1 = [math.radians(73.5),
#                      math.radians(64),
#                      math.radians(-118),
#                      math.radians(31),
#                      math.radians(3),
#                      math.radians(1)]
# arm_group.set_joint_value_target(lst_joint_angles_1)
# arm_group.plan()
# flag_plan_1=arm_group.go()

# #correcting the position to reach red pepper
# lst_joint_angles_2 = [math.radians(73.5),
#                      math.radians(66),
#                      math.radians(-110),
#                      math.radians(31),
#                      math.radians(3),
#                      math.radians(1)]
# arm_group.set_joint_value_target(lst_joint_angles_2)
# arm_group.plan()
# flag_plan_2=arm_group.go()

# #gripper close to capture
# hand_group.set_named_target("close")
# plan1=hand_group.go()

# #revert back to red basket position
# lst_joint_angles_3 = [math.radians(-20),
#                      math.radians(0),
#                      math.radians(0),
#                      math.radians(0),
#                      math.radians(0),
#                      math.radians(0)]
# arm_group.set_joint_value_target(lst_joint_angles_3)
# arm_group.plan()
# flag_plan_3=arm_group.go()

# #gripper open to drop
# hand_group.set_named_target("open")
# plan2=hand_group.go()

# ############################# YELLOW PEPPER #########################################

# #arm go to yellow pepper
# lst_joint_angles_4 = [math.radians(108),
#                      math.radians(54),
#                      math.radians(-42),
#                      math.radians(-1),
#                      math.radians(31),
#                      math.radians(0)]
# arm_group.set_joint_value_target(lst_joint_angles_4)
# arm_group.plan()
# flag_plan_4=arm_group.go()

# #correcting the position to reach yellow pepper
# lst_joint_angles_5 = [math.radians(103),
#                      math.radians(66),
#                      math.radians(-54),
#                      math.radians(-1),
#                      math.radians(31),
#                      math.radians(0)]
# arm_group.set_joint_value_target(lst_joint_angles_5)
# arm_group.plan()
# flag_plan_5=arm_group.go()


# #gripper close to capture
# hand_group.set_named_target("close")
# plan3=hand_group.go()

# #avoid collision
# lst_joint_angles_6 = [math.radians(108),
#                      math.radians(54),
#                      math.radians(-42),
#                      math.radians(-1),
#                      math.radians(31),
#                      math.radians(0)]
# arm_group.set_joint_value_target(lst_joint_angles_6)
# arm_group.plan()
# flag_plan_6=arm_group.go()


# #revert back to yellow basket position
# lst_joint_angles_7 = [math.radians(15),
#                      math.radians(0),
#                      math.radians(0),
#                      math.radians(0),
#                      math.radians(0),
#                      math.radians(0)]
# arm_group.set_joint_value_target(lst_joint_angles_7)
# arm_group.plan()
# flag_plan_7=arm_group.go()

# #gripper open to drop
# hand_group.set_named_target("open")
# plan4=hand_group.go()

rospy.sleep(1)
moveit_commander.roscpp_shutdown()

rospy.spin()
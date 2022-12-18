#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ KB-3983 ]
# Author List:		[ Nikhil S , Gokul,Anudeep,Suhail]
# Filename:			percepStack.py
# Functions:		
# 					[img_clbck,depth_clbck,image_processing,main]


####################### IMPORT MODULES #######################
import cv2 
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import tf
##############################################################

# Initialize Global variables
pose = []
pub_rgb = []
pub_depth =[]
################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


def img_clbck(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    rate = rospy.Rate(10)
    bridge  = CvBridge()
    cv_image  = bridge.imgmsg_to_cv2(img_msg,"bgr8")

    
    global pub_rgb #, add global variable if any

    ############################### Add your code here #######################################

    ##########################################################################################
    global pose
    pose = image_processing(cv_image)
    pub_rgb.publish(str(pose))

def depth_clbck(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    global pose
    global pub_depth
    rate = rospy.Rate(10)
    depth_val = []
    ############################### Add your code here #######################################
    bridge  = CvBridge()
    dept_image  = bridge.imgmsg_to_cv2(depth_msg,desired_encoding="passthrough")
    res = cv2.resize(dept_image,dsize =(1280,720),interpolation = cv2.INTER_CUBIC)
    for i in pose:
        depth_val.append(res[i[0],i[1]])
    ############################ Finding the Coordinates of Fruit with respect to camera
    x= pose[0][0]
    y = pose[0][1]
    X = depth_val[0]*((x-320.5)/554.3827128226441)
    Y = depth_val[0]*((y-240.5)/554.3827128226441)
    Z = depth_val[0]

    ######################### Publishing Transforms ######################################3
    br = tf.TransformBroadcaster()
    br.sendTransform((X,Y,Z),(0,0,0,1),rospy.Time.now(),'fruit','camera_depth_frame2')
    bk = tf.TransformBroadcaster()
    bk.sendTransform((X/5,Y/5,Z/5),(0,0,0,1),rospy.Time.now(),'fruit_2','camera_depth_frame2')

    bn = tf.TransformBroadcaster()
    bn.sendTransform((X,Y,Z),(0,0,0,1),rospy.Time.now(),'fruit_red','ebot_base')
    pub_depth.publish(str(depth_val))
        
    ##########################################################################################

    
   


def image_processing(image):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.  
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.

    Input Args:
    ------
    image: Converted image in cv2 format.

    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''
    pose_1= []
    ############### Write Your code to find centroid of the bell peppers #####################
    img_hsv = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    mask= cv2.inRange(img_hsv, (93,120,93), (113,255,255))
    mask1 = cv2.inRange(img_hsv, (108,140,0), (255,255,255))
    result = cv2.bitwise_and(image,image,mask =mask+mask1)
    blurred = cv2.GaussianBlur(result, (17, 17),0)
    kernel = np.ones((13,13))
    gradient = cv2.morphologyEx(blurred, cv2.MORPH_OPEN, kernel)
    edged = cv2.Canny(gradient,80,100)
    contours,_ = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    image_copy = image.copy()
    for c in contours:
        if cv2.contourArea(c) < 10:
            continue
        M= cv2.moments(c)
        if M["m00"]!=0:
            cX = int(M["m10"]/M["m00"])
            cY = int(M["m01"]/M["m00"])
        else:
            cX,cY = 0,0
        pose_1.append([cX,cY])
        cv2.circle(image_copy,(cX,cY),3,(255,0,0),-1)
    return pose_1
    ##########################################################################################
    



def main():
    '''
    MAIN FUNCTION

    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.

    NOTE: We have done the subscription only for one image, you have to iterate over 
    three images in the same script and publish the centroid and depth in the 
    same script for three images, calling the same callback function.

    '''

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####
    global pub_rgb
    global pub_depth
    rospy.init_node("percepStack", anonymous=True)
    sub_image_color_1 = rospy.Subscriber("/camera/color/image_raw2", Image, img_clbck)
    sub_image_depth_1 = rospy.Subscriber("/camera/depth/image_raw2", Image, depth_clbck)
    


    pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)

    ####################################################################################################
    rospy.spin()

if __name__ == '__main__':
    
    rospy.sleep(3)
    
    
    
    
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")





#! /usr/bin/env python3




# Team ID:			[ KB-3983 ]
# Author List:		[ Nikhil S , Gokul,Anudeep,Suhail]
# Filename:			maniStack.py		

####################### IMPORT MODULES #######################
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import actionlib
import math
import roslib 
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
##############################################################

#Initialisations
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('arm_manipulation',anonymous=True)
robot=moveit_commander.RobotCommander()
scene=moveit_commander.PlanningSceneInterface()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
execute_trajectory_client=actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
execute_trajectory_client.wait_for_server()
arm_group=moveit_commander.MoveGroupCommander("arm")
hand_group=moveit_commander.MoveGroupCommander("gripper")
rospy.sleep(1)

############################# RED PEPPER #########################################

#arm to detect red pepper
lst_joint_angles_1 = [math.radians(87.73),
                     math.radians(-0.841),
                     math.radians(-16.642955),
                     math.radians(-13.17922),
                     math.radians(-0.195),
                     math.radians(-0.0003)]
arm_group.set_joint_value_target(lst_joint_angles_1)
arm_group.plan()
flag_plan_1=arm_group.go()
listener = tf.TransformListener()
listener.waitForTransform("fruit", "ebot_base", rospy.Time(), rospy.Duration(4.0))
(trans,rot) = listener.lookupTransform('ebot_base', 'fruit', rospy.Time(0))
 
    
# to calculate the required position for end effector and reaching there.
red_pose = geometry_msgs.msg.Pose()
red_pose.position.x = trans[0]-0.01
red_pose.position.y = trans[1]-0.32
red_pose.position.z = trans[2]-0.12
(red_pose.orientation.x,red_pose.orientation.y,red_pose.orientation.z,red_pose.orientation.w) = quaternion_from_euler(2.833,0.0009,3.108)
pose_values = arm_group.get_current_pose().pose
arm_group.set_pose_target(red_pose)
flag_plan = arm_group.go()


#gripper close to capture
hand_group.set_named_target("close")
plan1=hand_group.go()


# #revert back to red basket position
lst_joint_angles_3 = [math.radians(-20),
                     math.radians(0),
                     math.radians(0),
                     math.radians(0),
                     math.radians(0),
                     math.radians(0)]
arm_group.set_joint_value_target(lst_joint_angles_3)
arm_group.plan()
flag_plan_3=arm_group.go()
hand_group.set_named_target("open")
plan2=hand_group.go()


### Arm to detect yellow pepper
lst_joint_angles_1 = [math.radians(114),
                     math.radians(11),
                     math.radians(-22),
                     math.radians(33),
                     math.radians(0),
                     math.radians(0)]
arm_group.set_joint_value_target(lst_joint_angles_1)
arm_group.plan()
flag_plan_1=arm_group.go()
listener.waitForTransform("fruit_2", "ebot_base", rospy.Time(), rospy.Duration(4.0))
(trans,rot) = listener.lookupTransform('ebot_base', 'fruit_2', rospy.Time(0))


# to calculate the required position for end effector and reaching there.
yellow_pose = geometry_msgs.msg.Pose()
yellow_pose.position.x = trans[0]+0.11
yellow_pose.position.y = trans[1]-0.25
yellow_pose.position.z = trans[2]+0.15
(yellow_pose.orientation.x,yellow_pose.orientation.y,yellow_pose.orientation.z,yellow_pose.orientation.w) = quaternion_from_euler(-2.604,-0.001,-2.7208)
print(trans)
print(rot)
pose_values = arm_group.get_current_pose().pose
arm_group.set_pose_target(yellow_pose)
flag_plan = arm_group.go()


# gripper close to capture
hand_group.set_named_target("close")
plan1=hand_group.go()

#revert back to yellow basket position
lst_joint_angles_7 = [math.radians(15),
                     math.radians(0),
                     math.radians(0),
                     math.radians(0),
                     math.radians(0),
                     math.radians(0)]
arm_group.set_joint_value_target(lst_joint_angles_7)
arm_group.plan()
flag_plan_7=arm_group.go()

# #gripper open to drop
hand_group.set_named_target("open")
plan4=hand_group.go()

rospy.sleep(1)
moveit_commander.roscpp_shutdown()

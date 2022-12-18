#!/usr/bin/env python3

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
pose_red = []
pose_yellow = []
pub_rgb = []
pub_depth =[]
low_yellow = np.array([93,120,93])
high_yellow = np.array([113,255,255])
low_red = np.array([111,140,0])
high_red = np.array([255,255,255])
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
    global pose_red 
    global pose_yellow
    pose_red = image_processing(cv_image,low_red,high_red)
    pose_yellow  = image_processing(cv_image,low_yellow,high_yellow)
    #print(pose_red,pose_yellow)
    

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
    global pose_red
    global pose_yellow
    global pub_depth
    rate = rospy.Rate(10)
    depth_val = []
    ############################### Add your code here #######################################
    bridge  = CvBridge()
    dept_image  = bridge.imgmsg_to_cv2(depth_msg,desired_encoding="passthrough")
    res = cv2.resize(dept_image,dsize =(1280,720),interpolation = cv2.INTER_CUBIC)
    if len(pose_red)>0:
        depth_val.append(res[pose_red[0][0],pose_red[0][1]])
        x= pose_red[0][0]
        y = pose_red[0][1]
        X = depth_val[0]*((x-320.5)/554.3827128226441)
        Y = depth_val[0]*((y-240.5)/554.3827128226441)
        Z = depth_val[0]
        if depth_val[0]<4:
            if depth_val[0]<1:
                br = tf.TransformBroadcaster()
                br.sendTransform((X,Y,Z),(0,0,0,1),rospy.Time.now(),'fruit_red','camera_depth_frame2')
            else :
                br = tf.TransformBroadcaster()
                br.sendTransform((X/5,Y/5,Z/5),(0,0,0,1),rospy.Time.now(),'fruit_red','camera_depth_frame2')
            pub_rgb.publish(str('red'))
        
    elif len(pose_yellow)>0:
        depth_val.append(res[pose_yellow[0][0],pose_yellow[0][1]])
        x= pose_yellow[0][0]
        y = pose_yellow[0][1]
        X = depth_val[0]*((x-320.5)/554.3827128226441)
        Y = depth_val[0]*((y-240.5)/554.3827128226441)
        Z = depth_val[0]
        
        if depth_val[0]<4:
            if depth_val[0]<1:
                br = tf.TransformBroadcaster()
                br.sendTransform((X,Y,Z),(0,0,0,1),rospy.Time.now(),'fruit_yellow','camera_depth_frame2')
            else :
                br = tf.TransformBroadcaster()
                br.sendTransform((X/5,Y/5,Z/5),(0,0,0,1),rospy.Time.now(),'fruit_yellow','camera_depth_frame2')
            pub_rgb.publish(str('yellow'))
        
    else :
        pub_rgb.publish(str('none'))
    

    
    '''
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
        
    ##########################################################################################'''

    
   


def image_processing(image,low,high):
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
    mask= cv2.inRange(img_hsv,low,high)
    result = cv2.bitwise_and(image,image,mask =mask)
    blurred = cv2.GaussianBlur(result, (17, 17),0)
    ret, thresh = cv2.threshold(blurred, 100, 255,cv2.THRESH_BINARY)
    img_gray = cv2.cvtColor(thresh, cv2.COLOR_RGB2GRAY)
    ret, thresh2 = cv2.threshold((img_gray), 20, 255,cv2.THRESH_BINARY)
    #th3 = cv2.adaptiveThreshold(img_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,3)
    kernel = np.ones((13,13))
    gradient = cv2.morphologyEx(thresh2, cv2.MORPH_OPEN, kernel)
    edged = cv2.Canny(gradient,80,100)
    contours,_ = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    image_copy = image.copy()
    for c in contours:
        if cv2.contourArea(c) < 500:
            continue
        M= cv2.moments(c)
        if M["m00"]!=0:
            cX = int(M["m10"]/M["m00"])
            cY = int(M["m01"]/M["m00"])
        else:
            cX,cY = 0,0
        pose_1.append([cX,cY])
        cv2.circle(image_copy,(cX,cY),3,(255,0,0),-1)

    cv2.drawContours(image_copy, contours, -1, (0, 255, 0), 2)
    cv2.imshow('ima',image_copy)
    cv2.waitKey(1)
    
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
    #sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
    #sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)
    
    


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

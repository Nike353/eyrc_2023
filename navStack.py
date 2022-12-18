#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


global state
change  = 0
state=0
complete = []
last_err_fright=0
last_err_bright=0
last_err_fleft=0
last_err_bleft=0
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub1=rospy.Publisher('/nav_mani',String,queue_size=10)

regions={
    'bright':0,
    'fright':0,
    'front':0,
    'fleft':0,
    'bleft':0
}

def laser_callback(msg):
    global regions
    range_max=8
    regions = {
        'bright':min(msg.ranges[0],range_max),
        'fright':min(msg.ranges[179],range_max),
        'front':min(msg.ranges[359],range_max),
        'fleft':min(msg.ranges[539],range_max),
        'bleft':min(msg.ranges[719],range_max)
    }

def left_steering(err_bleft,err_fleft):       # left wall follow PD controller
    global last_err_fleft
    global last_err_bleft
    
    derr_bleft = err_bleft - last_err_bleft
    derr_fleft = err_fleft - last_err_fleft
    velocity_msg = Twist()
    velocity_msg.linear.x = 0.2 + 0.7*(err_fleft) + 10*(derr_fleft)
    velocity_msg.angular.z = 0.7*(err_bleft) + 10*derr_bleft + 1.1*(err_fleft)
    last_err_bleft = err_bleft
    last_err_fleft = err_fleft
    return velocity_msg

def right_steering(err_bright,err_fright):       # left wall follow PD controller
    global last_err_fright
    global last_err_bright
    
    derr_bright = err_bright - last_err_bright
    derr_fright = err_fright - last_err_fright
    velocity_msg = Twist()
    velocity_msg.linear.x = 0.4 + 0.4*(err_fright) + 10*(derr_fright)
    velocity_msg.angular.z = 0.5*(err_bright) + 9*derr_bright + 1.1*(err_fright)
    last_err_bright = err_bright
    last_err_fright = err_fright
    return velocity_msg

def mani_nav_callback(data):
    global complete
    complete = data.data

def percep_nav_callback(data):
    global change
    velocity_msg=Twist()
    if data.data=="yellow" or data.data=="red":
        change  = 1
        rospy.sleep(0.01)
        if data.data=="yellow":
            pub1.publish("PLUCK_YELLOW")
        else:
            velocity_msg.linear.x=-0.01
            pub1.publish("PLUCK_RED")
    else :
        change = 0
        pub1.publish("None")
        


def control_loop():
    rospy.init_node('ebot_controller')
    
   
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/mani_nav',String,mani_nav_callback)
    rospy.Subscriber('/center_rgb',String,percep_nav_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    global state
    global regions
    global complete
    rospy.sleep(8)


    while not rospy.is_shutdown():
        
        if state==0:
            if regions['bleft']<0.8:
                state=1
            else:
                velocity_msg.linear.x=0.5
                velocity_msg.angular.z=0
                pub.publish(velocity_msg)
        elif state==1:
            if regions['front']>1.275:
                if change ==1:
                    while True:
                        if complete!= "OKIE":
                            velocity_msg.linear.x=0.0
                            velocity_msg.angular.z=0
                            pub.publish(velocity_msg)
                        else:
                            break
                        
                    
                velocity_msg= left_steering(regions['bleft']-0.66,regions['fleft']-0.8)
                pub.publish(velocity_msg)
            else:
                state =2
        else :
            velocity_msg.linear.x=0.0
            velocity_msg.angular.z=0
            pub.publish(velocity_msg)

        # if state==0:
        #     if regions['bleft']<0.8:
        #         state=1
        #     else:
        #         velocity_msg.linear.x=0.5
        #         velocity_msg.angular.z=0
        #         pub.publish(velocity_msg)
        # elif state==1:
        #     if regions['front']>1.275:
        #         left_steering(regions['bleft']-0.63,regions['fleft']-0.78)
        #         pub.publish(velocity_msg)
        #     else:
        #         state=2
        # elif state==2:
        #     if regions['front']<4:
        #         velocity_msg.linear.x=0.75
        #         velocity_msg.angular.z=-2
        #         pub.publish(velocity_msg)
        #     else:
        #         state=3
        # elif state==3:
        #     if (((regions['bright']>4 or regions['fright']>0.85)) and regions['front']<3):
        #         velocity_msg.linear.x = 0.5
        #         velocity_msg.angular.z = 0
        #         pub.publish(velocity_msg)
        #         rospy.sleep(1)
        #         state = 4 
        #     else:
        #         if (regions['front']>3):
        #             velocity_msg.linear.x = 0.5 + 0.5*(regions['fright']-0.78)
        #             velocity_msg.angular.z = -(0 + 0.5*(regions['bright']-0.63) + 0.8*(regions['fright']-0.8))
        #             pub.publish(velocity_msg)
        #         else :
        #             velocity_msg.linear.x = 0.4 + 0.4*(regions['fright']-0.78)
        #             velocity_msg.angular.z = -(0 + 0.5*(regions['bright']-0.63) + 0.8*(regions['fright']-0.8))
        #             pub.publish(velocity_msg)
        # elif state==4:
        #     while regions['front']<1.2:
        #         velocity_msg.linear.x = 1
        #         velocity_msg.angular.z = 3.5
        #         pub.publish(velocity_msg)
        # #    
        # # elif state==5:
        # #     velocity_msg.linear.x=3
        # #     velocity_msg.angular.z=0
        # #     pub.publish(velocity_msg)

                
        # # elif state==5:
        # #     if regions['front']>7:
        # #         velocity_msg.linear.x=1
        # #         velocity_msg.angular.z=0
        # #         pub.publish(velocity_msg)
        # rate.sleep()
        
if __name__ == '__main__':
    try:
        state=0
        control_loop()
    except rospy.ROSInterruptException:
        pass
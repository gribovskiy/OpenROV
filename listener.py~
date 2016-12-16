#!/usr/bin/env python
# 

import rospy
from std_msgs.msg import String
from openrov.msg import navdata
#from geometry_msg.msg import Twist
import geometry_msgs.msg
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
import numpy as np



motor_x=0
motor_y=0


has_goal=0
goal=0
initial_head=0
loop=0
init_cam=0



def callback_image(data):
    #rospy.loginfo(rospy.get_caller_id() + '   1 I heard %s', data.data)
    
    global has_goal
    global goal
    global init_cam
    
    motor_x=float(data.data.split(" ")[0])
    motor_y=float(data.data.split(" ")[1])
    if (init_cam>70):
        if (motor_x!=0) and (has_goal==0) :
            print "new goal"
        
            goal=float(motor_x/100)
            has_goal=1
    else :
        print "Init_cam ",init_cam,"/70"
      
    #print "call 1",has_goal
    
    #Rate should be slower than the sender
    #rate = rospy.Rate(3.0)
    #rate.sleep()
    
    
def callback_control(navdata):
    #rospy.loginfo(rospy.get_caller_id() + '   2 I heard %s', navdata)
    #print navdata.heading, " goal ", 180
    
    global has_goal
    global goal
    global initial_head
    global loop
    global init_cam
    
    print navdata.heading
   
   
   
   
    plt.axis([0,200,0,200])
    plt.ion()
    plt.show()

    x = navdata.heading
    y=navdata.heading
   
    plt.plot(x, y)
    plt.draw()
    plt.pause(0.001)
    
    
    
    loop+=1
    if loop<10:
        initial_head=navdata.heading
        print "Initialisation : ",loop, "/10 ", initial_head
        exit()
    if loop>10000:
        loop=100
        
    init_cam+=1
    if init_cam>10000:
        init_cam=0
	    
    diff=(initial_head+goal)-navdata.heading
    if diff>180:
	diff-=180
    if diff<-180:
	diff+=180
	
	#If goal is reached
    if abs(diff)<1 and has_goal==1:
        init_cam=0
    	diff=0
    	has_goal=0
    	goal=0
    	
    diff=diff/100
    if has_goal:
        print "But : ",initial_head+goal," actuel : ",navdata.heading, " diff : ",diff
    
   
    #No goal, so update initial heading
    if has_goal==0:
       initial_head=navdata.heading
    
    if has_goal==1:
	    print "J'ai un goal : ",goal
    else:
	    print "No goal"
	
    pub = rospy.Publisher('/openrov/cmd_rate', Twist, queue_size=0)
    cmd = geometry_msgs.msg.Twist()
    
    cmd.linear.x = 0.00
    cmd.linear.y = 0.00
    cmd.linear.z = 0.00
    cmd.angular.x = 0.00
    cmd.angular.y = 0.00
    if diff>0.5:
        diff=0.5
    if diff<-0.5:
        diff=-0.5
    cmd.angular.z = diff
    
    #Not too offen to avoid overflowing the motor control
    if loop%10==1:
        print "Sending command"
        #pub.publish(cmd)
        print "Not sending because commented"
        
    pub = rospy.Publisher('goal_heading', String, queue_size=0)

    if init_cam>70:
        stabilized=1
    else:
        stabilized=0
    hello_str = str(has_goal)+" "+str(diff)+" "+str(stabilized)+" "+str(navdata.heading)+" "+str(goal)
    #rospy.loginfo(hello_str)
    pub.publish(hello_str)
    
    #rate = rospy.Rate(3.0)
    #rate.sleep()
    
    


def listener():


    #rospy.init_node('control_heading', anonymous=True)

    rospy.Subscriber('image_feature', String, callback_image)
    rospy.Subscriber('/openrov/navdata', navdata, callback_control)
    

    rospy.init_node('goal_heading', anonymous=True)
    

    rospy.spin()

    

if __name__ == '__main__':
    listener()

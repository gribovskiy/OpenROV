#!/usr/bin/env python
# 

import rospy
from std_msgs.msg import String
from openrov.msg import navdata
import geometry_msgs.msg
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
import numpy as np



goal_x=0
goal_y=0


has_goal=0 #Is he triyng to reache an angle
goal=0 #Angle he is trying to reach
initial_head=0 #Initial angle, used to calculate the new one
loop=0 #How many loop have we done. Used as a timmer

#Remember the algorithm used to isolate fish uses background stabilization, so it needs time
# to acquire the background if it changed
init_cam=0 #If we just moved, let the OpenROV still for a few moment
frame_to_stabilize=70 #How much frame are needed so the image is stable again (value is empirical)
precision=1 #Precision to achieve while rotating, in degree


#Called every times "open_rov" send a new average fish position
#It's goal is to get that position and put it in the global variable "goal" (and update "has_goal")
def callback_image(data):
    global has_goal
    global goal
    global init_cam
    global frame_to_stabilize
    
    goal_x=float(data.data.split(" ")[0])
    goal_y=float(data.data.split(" ")[1])
    if (init_cam>frame_to_stabilize):
        #We only care about X because it is the one on the XY plan (rotation around Z)
        #OpenROV can't rotate n the XZ plan (rotation around Y)
        if (goal_x!=0) and (has_goal==0) :
            print "new goal"
            goal=float(goal_x/100)
            has_goal=1
    else :
        #Wait for image to stabilize
        print "Init_cam ",init_cam,"/",frame_to_stabilize
      
   
#Called every times "navdata" send a new gyro value
#It's goal is to reach the "goal" position, and then update "has_goal"
def callback_control(navdata):
    global has_goal
    global goal
    global initial_head
    global loop
    global init_cam
    global frame_to_stabilize
    print navdata.heading
   
   
   
    #Used to plot the datas, now in script "plot"
    #plt.axis([0,200,0,200])
    #plt.ion()
    #plt.show()
    #x = navdata.heading
    #y=navdata.heading
   
    #plt.plot(x, y)
    #plt.draw()
    #plt.pause(0.001)
    
    
    loop+=1
    #Wait some times for the gyro to initialize 
    #10 is empirical and should theorically not be touched
    if loop<10:
        initial_head=navdata.heading
        print "Init : ",loop, "/10 ", initial_head
        exit()
    #Avoid overflow for very long usages, only for safety
    if loop>1000:
        loop=1000
        
    init_cam+=1
       
    #Avoid overflow for very long usages, only for safety
    if init_cam>1000:
        init_cam=1000
	    
    diff=(initial_head+goal)-navdata.heading
    #Put back in [-180,180] range
    if diff>180:
	diff-=180
    if diff<-180:
	diff+=180
	
    #If goal is reached with a precision of 1 degree (empirical)
    global precision
    if abs(diff)<precision and has_goal==1:
        init_cam=0
    	diff=0
    	has_goal=0
    	goal=0
    	
    #Diff is a percentage, convert it back to be applied to the motors
    diff=diff/100
    if has_goal:
        print "Goal : ",initial_head+goal," actual : ",navdata.heading, " diff : ",diff
    
   
    #No goal, so update initial heading
    if has_goal==0:
       initial_head=navdata.heading
    
    if has_goal==1:
	    print "Got a goal : ",goal
    else:
	    print "No goal"
	
    #Command to be sent
    pub = rospy.Publisher('/openrov/cmd_rate', Twist, queue_size=0)
    cmd = geometry_msgs.msg.Twist()
    
    cmd.linear.x = 0.00
    cmd.linear.y = 0.00
    cmd.linear.z = 0.00
    cmd.angular.x = 0.00
    cmd.angular.y = 0.00
    #Only for safety reasons, that way even if a problem occure, it shouldn't go too fast
    if diff>0.5:
        diff=0.5
    if diff<-0.5:
        diff=-0.5
    cmd.angular.z = diff
    
    #Motor control can't handdle command the same speed as the gyro send them
    #As it is 10 times slower, is is only sent 1/10th of the time, else it accumulate delay
    if loop%10==1:
        print "Sending command"
        pub.publish(cmd)

        
    pub = rospy.Publisher('goal_heading', String, queue_size=0)

    if init_cam>frame_to_stabilize:
        stabilized=1
    else:
        stabilized=0
    send_str = str(has_goal)+" "+str(diff)+" "+str(stabilized)+" "+str(navdata.heading)+" "+str(goal)

    #Variables to be display on the main controller (listener.py)
    pub.publish(send_str)

    


def listener():

    #Read from
    rospy.Subscriber('image_feature', String, callback_image) #average position of the fishs
    rospy.Subscriber('/openrov/navdata', navdata, callback_control) #control of the OpenROV

    #Write to
    rospy.init_node('goal_heading', anonymous=True) #targeted angle
    

    rospy.spin()

    

if __name__ == '__main__':
    listener()

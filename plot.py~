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

avoid_frame=3
nb_data=100

loop_plot=0

x=np.arange(1,nb_data)

heading=np.arange(1,nb_data)
roll=np.arange(1,nb_data)
pitch=np.arange(1,nb_data)
yaw=np.arange(1,nb_data)

    
    
def callback_control(navdata):
    #rospy.loginfo(rospy.get_caller_id() + '   2 I heard %s', navdata)
    #print navdata.heading, " goal ", 180
    
    global loop_plot, x
    
    loop_plot+=1
    

    global heading, roll, pitch, yaw
    
    
    print navdata.heading,navdata.roll,navdata.pitch,navdata.yaw, loop_plot

    heading=np.roll(heading,-1) 
    roll=np.roll(roll,-1)
    pitch=np.roll(pitch,-1)
    yaw=np.roll(yaw,-1)
    
    heading[-1]=navdata.heading
    roll[-1]=navdata.roll
    pitch[-1]=navdata.pitch
    yaw[-1]=navdata.yaw
    
   

    if loop_plot<avoid_frame:
        return
    loop_plot=0

    #plt.scatter(loop_plot, navdata.heading)
    plt.clf()
 
    #plt.plot(heading)
    #heading is exactly the same as yaw
    
    plt.plot(roll)
    plt.plot(pitch)
    plt.plot(yaw)
    plt.pause(0.001)
    
    


    
    


    
   
        
    
def plot():


    #rospy.init_node('control_heading', anonymous=True)

    #rospy.Subscriber('image_feature', String, callback_image)
    rospy.Subscriber('/openrov/navdata', navdata, callback_control)

    rospy.init_node('plot', anonymous=True)
    rospy.spin()

    

if __name__ == '__main__':
    plot()

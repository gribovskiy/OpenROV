#!/usr/bin/env python
# 

"""
Plot de 3 axis gyro with matplotlib, and also print them to the terminal -> receive (navdata) from channel
"""
__author__ =  'Jonathan Muller <jonathan.muller at epfl.ch>'
__version__=  '1.0'
__license__ = 'BSD'

import rospy
from std_msgs.msg import String
from openrov.msg import navdata
#from geometry_msg.msg import Twist
import geometry_msgs.msg
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
import numpy as np

avoid_frame=3 #Too much frame cause lag. Display only 1/avoid_frame of the frame
nb_data=100 #X range

loop_plot=0 #How many loop have we done. Used as a timmer

x=np.arange(1,nb_data) #X axis

heading=np.arange(1,nb_data) #Y1
roll=np.arange(1,nb_data) #Y2
pitch=np.arange(1,nb_data) #Y3
yaw=np.arange(1,nb_data) #Y4

    
#Called every new gyro data
def callback_control(navdata):
    global loop_plot, x
    global heading, roll, pitch, yaw
    loop_plot+=1
    
    print navdata.heading,navdata.roll,navdata.pitch,navdata.yaw, loop_plot

    #Shift from 1, as the new value will come
    heading=np.roll(heading,-1) 
    roll=np.roll(roll,-1)
    pitch=np.roll(pitch,-1)
    yaw=np.roll(yaw,-1)
    
    #Put the new value
    heading[-1]=navdata.heading
    roll[-1]=navdata.roll
    pitch[-1]=navdata.pitch
    yaw[-1]=navdata.yaw
    
   
    #Jump some frame to avoid lag
    if loop_plot<avoid_frame:
        return
    loop_plot=0

    #plt.scatter(loop_plot, navdata.heading)
    plt.clf()

    plt.plot(roll)
    plt.plot(pitch)
    plt.plot(yaw)
    #plt.plot(heading)
    #heading is exactly the same as yaw because of sensor fusion. No need to display both
    plt.pause(0.001)
    
    


    
    


    
   
        
    
def plot():


    #rospy.init_node('control_heading', anonymous=True)

    #rospy.Subscriber('image_feature', String, callback_image)
    rospy.Subscriber('/openrov/navdata', navdata, callback_control)

    rospy.init_node('plot', anonymous=True)
    rospy.spin()

    

if __name__ == '__main__':
    plot()

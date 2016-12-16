#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
from std_msgs.msg import String

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False


# import the necessary packages
import numpy as np
import argparse
import glob

has_goal=0
command=0
stabilized=0
heading=0



class image_feature:
    #loop=0
    def __init__(self):
        self.frame_drop=0
        self.loop=0
        self.untouched=0
        self.frame1=0
        self.frame2=0
        self.frame1_gray=0
        self.frame2_gray=0
        self.analyse=0
        
        self.frame_to_init=70
        
        #Change with real size IF you changes the defaults parameters
        self.h=216
        self.w=386
        
        self.x_moy=0
        self.y_moy=0
        
        self.init_frame=0
        self.nb_fish=0
        
        self.bg_fact=0.95 #Background_factor, from 0 to 1, higher=keeps the image longer
        self.min_light_fact=45 #The higher it is, the least the image is impacted by variance
                               #but is also less sharp
        self.blur_fact=7 #Size of the blur kernel
        self.thresh_fact=60 #Threshold
        
      

        
        self.background=0

        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",CompressedImage)
        # self.bridge = CvBridge()

        # subscribed Topic
        #self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size = 1)
        self.subscriber = rospy.Subscriber("/camera/visible/image/compressed", CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"


    def callback(self, ros_data):
        
    	pub = rospy.Publisher('image_feature', String, queue_size=10)
    	#rospy.init_node('talker', anonymous=True)
    	#hello_str = "hello world %s" % rospy.get_time()
    	
    	try:
    	    self.h=self.frame1.shape[0]
    	    self.w=self.frame1.shape[1]
    	except:
    	    print "Still not init"
    	
    	percent_x=(self.x_moy-self.w/2)*1000/(self.w/2)
    	percent_y=(self.y_moy-self.h/2)*1000/(self.h/2)
    	
    	
    	hello_str = str(percent_x)+" "+str(percent_y)
    	
    	if self.loop>self.frame_to_init:
    	    #print hello_str
    	    pub.publish(hello_str)
    	else :
    	    print "Waiting for frame to stabilize ",self.loop,"/"+str(self.frame_to_init)

        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format
            
        pressed="a faire"
        keyboard=cv2.waitKey(5)
	if keyboard == ord('q'):
	    self.bg_fact+=0.01
	if keyboard == ord('w'):
	    self.bg_fact-=0.01
	if keyboard == ord('a'):
	    self.min_light_fact+=1
	if keyboard == ord('s'):
	    self.min_light_fact-=1
	if keyboard == ord('d'):
	    self.blur_fact+=2
	if keyboard == ord('f'):
	    self.blur_fact-=2
	if keyboard == ord('e'):
	    self.thresh_fact+=1
	if keyboard == ord('r'):
            self.thresh_fact-=1
            
        if self.bg_fact<0.5:
            self.bg_fact=0.5
        if self.bg_fact>0.99:
            self.bg_fact=0.99
        if self.min_light_fact<1:
            self.min_light_fact=1
        if self.min_light_fact>200:
            self.min_light_fact=200
        if self.blur_fact<1:
            self.blur_fact=1
        if self.blur_fact>51:
            self.blur_fact=51
        if self.thresh_fact>250:
            self.thresh_fact=250
        if self.thresh_fact<10:
            self.thresh_fact=10
            
        
            
        affiche = np.zeros((500,250,3), np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(affiche,"Use key to change factors",(10,20), font, 0.5,(255,255,255),2)
        
        cv2.putText(affiche,"Key pressed : "+str(pressed)+" ",(10,40), font, 0.5,(255,255,255),2)
        cv2.putText(affiche,"Background : "+str(self.bg_fact)+" (q w)",(10,60), font, 0.5,(255,255,255),2)
        cv2.putText(affiche,"Min light : "+str(self.min_light_fact)+" (a s)",(10,80), font, 0.5,(255,255,255),2)
        cv2.putText(affiche,"Blur kernel : "+str(self.blur_fact)+" (d f)",(10,100), font, 0.5,(255,255,255),2)
        cv2.putText(affiche,"Nb. of fish : "+str(self.nb_fish)+" (e r)",(10,120), font, 0.5,(255,255,255),2)
        cv2.putText(affiche,"Fish position L<->R : "+str(percent_x/10)+"%",(10,140), font, 0.5,(255,255,255),2)
        if(self.loop<self.frame_to_init):
            cv2.putText(affiche,"Init : "+str(self.loop)+"/"+str(self.frame_to_init),(10,160), font, 0.5,(0,0,255),2)
        else:
            if self.nb_fish>0:
	        cv2.line(affiche,(int(250/2), 160),(int(250/2+percent_x/10),160),(255,0,0),5)
	        cv2.circle(affiche, (250/2, 160), 3, (0, 255, 0), -1)
	    else:
	        cv2.putText(affiche,"No fish",(10,160), font, 0.5,(255,255,255),2)
        global has_goal
        global command
        global stabilized
        global heading

	if has_goal:
            cv2.putText(affiche,"Target is at L<->R: "+str(command*200),(10,180), font, 0.5,(255,255,255),2)
            cv2.line(affiche,(int(250/2), 200),(int(250/2+command*200),200),(255,0,0),5)
            cv2.circle(affiche, (250/2, 200), 3, (0, 255, 0), -1)

        else:
            cv2.putText(affiche,"Waiting for target : ",(10,180), font, 0.5,(255,255,255),2)
        
        cv2.putText(affiche,"Command : "+str(command),(10,220), font, 0.5,(255,255,255),2)
        if has_goal:
            cv2.putText(affiche,"Image is not used",(10,240), font, 0.5,(255,255,255),2)
        else:
            if stabilized:
                cv2.putText(affiche,"Image is stable",(10,240), font, 0.5,(0,255,0),2)
            else:
                cv2.putText(affiche,"Image is stabilizing",(10,240), font, 0.5,(0,0,255),2)
        #cv2.putText(affiche,"Heading : "+str(command),(10,260), font, 0.5,(255,255,255),2)

        #### direct conversion to CV2 ####
        
        
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.frame_drop+=1
        DROP=3
        #Drop every "DROP" frame
        #So reduce framerate by "DROP" (to avoid buffer fill and latency)
      
        
        
        if self.frame_drop<DROP:
            return
        else:
            self.frame_drop=0
           
            
            
        bg_fact=self.bg_fact
        min_light_fact=self.min_light_fact
        blur_fact=self.blur_fact
        thresh_fact=self.thresh_fact

            

	
        
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.resize(image_np, (0,0), fx=0.3, fy=0.3) 
        
        
        (h, w) = image_np.shape[:2]
	center = (w / 2, h / 2)
	M = cv2.getRotationMatrix2D(center, 180, 1.0)
	
	image_np = cv2.warpAffine(image_np, M, (w, h))
        
               
        self.frame1=image_np
        self.frame1_gray=cv2.cvtColor(self.frame1, cv2.COLOR_BGR2GRAY)
      
        blur = cv2.GaussianBlur(self.frame1,(blur_fact,blur_fact),0)
        if(self.loop==0):
       	    self.frame2=self.frame1
       	    self.frame2_gray=self.frame1_gray

       	    self.background=blur
       	    
       	    self.loop+=1
       	    return 0
       	    
       
       	self.background = cv2.addWeighted(self.background,bg_fact,blur,1-bg_fact,0)

       	 
        #print self.frame1_gray.shape , self.frame2_gray.shape
        diff=cv2.subtract(blur,self.background) 
        
        gray=cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        
        minimum = np.amin(gray)
        maximum = np.amax(gray)
        
        im_range=gray
        im_range = cv2.addWeighted(im_range,255/(maximum+min_light_fact),im_range,0,0)

        

        ret,thresh = cv2.threshold(im_range,thresh_fact,255,cv2.THRESH_BINARY)        
        kernel = np.ones((5,5),np.uint8)
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)  
        
        
        image=self.frame1

        cnts = cv2.findContours(opening.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        cnts =  cnts[0]
        
        nb_fish=0
        
        self.x_moy=0
        self.y_moy=0

        for c in cnts:
	    # compute the center of the contour
	    M = cv2.moments(c)
	    if M["m00"]==0:
	        continue
	    cX = int(M["m10"] / M["m00"])
	    cY = int(M["m01"] / M["m00"])
 
	    # draw the contour and center of the shape on the image
	    cv2.drawContours(image, [c], -1, (0, 255, 0), 1)
	    cv2.circle(image, (cX, cY), 3, (255, 0, 0), -1)
	    cv2.putText(image, "Fish", (cX - 20, cY - 20),
	    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
	    
	    if (cX<self.w and cY < self.h):
	        self.x_moy+=cX
	        self.y_moy+=cY
	    
	        nb_fish+=1

	self.nb_fish=nb_fish
	if (nb_fish>0):
	    print "Detected ",nb_fish, " fishs"
	    self.x_moy=self.x_moy/nb_fish
	    self.y_moy=self.y_moy/nb_fish
	    print "Centroid of fishs ",self.x_moy," ", self.y_moy
	    cv2.circle(image, (self.x_moy, self.y_moy), 10, (255, 255, 0), -1)
	    cv2.putText(image, "Average", (self.x_moy - 20, self.y_moy - 20),
	    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
	else :
	    print "No fish detected"
	    self.x_moy=self.w/2
        self.y_moy=self.h/2
            

        
        #cv2.imshow("vrai",image_np)
       	#cv2.moveWindow("vrai", 0, 10)

        
        cv2.imshow("image",image)
        cv2.moveWindow("image", 500, 600)
        
        
        vis = np.concatenate((thresh,closing,opening), axis=0)
        cv2.imshow("Parameters",vis)
        cv2.moveWindow("vis", 0, 10)
        
        cv2.imshow("affiche",affiche)
        cv2.moveWindow("affiche", 450, 10)
        

 
     	
        #print "Loop : ",self.loop

        
        self.frame2=self.frame1
        self.frame2_gray=self.frame1_gray
        self.loop+=1
        if self.loop>10000:
            self.loop=1000

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        
        
        
        #self.subscriber.unregister()
        
def callback_head(navdata):
    #rospy.loginfo(rospy.get_caller_id() + '   2 I heard %s', navdata)
    #print navdata.heading, " goal ", 180
    global has_goal
    global command
    global stabilized
    global heading

    has_goal=int(navdata.data.split(" ")[0])
    command=float(navdata.data.split(" ")[1])
    stabilized=int(navdata.data.split(" ")[2])
    heading=float(navdata.data.split(" ")[3])


def main(args):

    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    rospy.Subscriber('goal_heading', String, callback_head)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

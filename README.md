# OpenROV
Keywords :
OpenROV, ROS, OpenCV, integration

The goal of this project is to integrate ROS with OpenROV to follow fishs shoals.

talker :
Just an exemple of a talker with ROS. No goal except to have the structure. -> send (talker) to channel
plot :
Plot de 3 axis gyro with matplotlib, and also print them to the terminal -> receive (navdata) from channel
open_rov :
Get the average position of the fish through OpenCV -> receive (image) and send (image_feature) to channel 
listener :
With the average fish position, send command to the motors -> receive (image_feature) and send (cmdrate) to channel

Contact : jonathan.muller12@gmail.com

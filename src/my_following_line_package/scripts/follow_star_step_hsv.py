#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveKobuki

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.movekobuki_object = MoveKobuki()

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 20
        #crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        
        crop_img = cv_image[int(height/2)+descentre : int(height/2)+(descentre+rows_to_watch), 1:width]


        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # Define the Yellow Colour in HSV
        #RGB
        #[[[222,255,0]]]
        #BGR
        #[[[0,255,222]]]
        """
        To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
        >>> yellow = np.uint8([[[B,G,R ]]])
        >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >>> print( hsv_yellow )
        [[[ 34 255 255]]
        """

        
        # For yellow line uncomment these lines

        # Threshold the HSV image to get only yellow colors
        #lower_yellow = np.array([20,100,100])
        #upper_yellow = np.array([50,255,255])
      
        # LOOSE color detection: 
        # lower_yellow = np.array([0,50,50])
        # upper_yellow = np.array([255,255,255])

        # STRICT color detection: 
        #lower_yellow = np.array([33,254,254])
        #upper_yellow = np.array([36,255,255])
          
        # mask = cv2.inRange(hsv, lower_yellow, upper_yellow)


        # Blue HSV value -> [[[111 243 233]]]

        lower_blue = np.array([80,100,100])
        upper_blue = np.array([130,255,255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        

        """
        # For Red star uncomment these lines and rotate the robot towards the red star

        # Red HSV value -> [[[  0 255 222]]]

        lower_red = np.array([0,100,100])
        upper_red = np.array([20,255,255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        """

        """
        # Green HSV value ->[[[ 60 255 255]]]

        # lower_green = np.array([50,100,100])
        # upper_green = np.array([70,255,255])

        # mask = cv2.inRange(hsv, lower_green, upper_green)
        """
        

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)
        
        cv2.waitKey(1)
        
        
        error_x = cx - width / 2;
        twist_object = Twist();
        twist_object.linear.x = 0.2;
        twist_object.angular.z = -error_x / 100;
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
        # Make it start turning
        self.movekobuki_object.move_robot(twist_object)
        
    def clean_up(self):
        self.movekobuki_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    
    line_follower_object = LineFollower()
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    main()
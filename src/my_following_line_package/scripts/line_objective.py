#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class FollowColor():
    
    def objective():
        # Initialize the ROS node
        rospy.init_node('line_objective', anonymous=True)
    
        # Create a publisher on the topic '/objective'
        pub = rospy.Publisher('/objective', String, queue_size=10)
    
        rospy.loginfo("Node started. Enter messages to publish or 'exit' to quit.")
    
        while not rospy.is_shutdown():
        # Ask the user for input
            user_input = input("Enter message to publish (or type 'exit' to quit): ")
        
            if user_input.lower() == 'exit':
                rospy.loginfo("Exiting follow_color node.")
                break
        
            # Publish the user's input
            pub.publish(user_input)
            rospy.loginfo(f"Published: {user_input}")


if __name__ == '__main__':
    try:
        FollowColor.objective()
    except rospy.ROSInterruptException:
        pass

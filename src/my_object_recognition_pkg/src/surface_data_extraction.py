#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

class SurfaceDataExtractor:
    def __init__(self):
        self.table_height = 0.8  # Default table height in meters
        self._error_height = 0.1  # Height tolerance in meters

        rospy.init_node('surface_data_extraction', anonymous=True)
        rospy.Subscriber('/surface_objects', Marker, self.marker_callback)
        rospy.loginfo("Surface Data Extractor initialized. Listening to /surface_objects topic.")

    def update_table_height(self, new_table_height):
        """Update the table height dynamically."""
        self.table_height = new_table_height

    def look_for_table_surface(self, z_value):
        """Check if the given z_value corresponds to the table height."""
        delta_min = self.table_height - self._error_height
        delta_max = self.table_height + self._error_height
        return delta_min <= z_value <= delta_max

    def marker_callback(self, marker):
        """Process the Marker and filter table surfaces."""
        if "surface" in marker.ns and "object" not in marker.ns:
            z_value = marker.pose.position.z
            if self.look_for_table_surface(z_value):
                rospy.loginfo(f"Detected table surface: {marker.ns}")
                rospy.loginfo(f"Position: {marker.pose.position}")
                rospy.loginfo(f"Orientation: {marker.pose.orientation}")

    def run(self):
        """Run the ROS node."""
        rospy.spin()

if __name__ == "__main__":
    extractor = SurfaceDataExtractor()
    extractor.run()

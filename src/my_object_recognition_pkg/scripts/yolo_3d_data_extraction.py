#!/usr/bin/env python3

import rospy
from darknet_ros_3d.msg import BoundingBox3DArray
import csv

# Function to calculate the center of the bounding box
def calculate_center(bbox):
    center_x = (bbox.min_x + bbox.max_x) / 2
    center_y = (bbox.min_y + bbox.max_y) / 2
    center_z = (bbox.min_z + bbox.max_z) / 2
    return center_x, center_y, center_z

# Callback function for the subscriber
def bounding_boxes_callback(data):
    target_class = "elephant"  # Change to the object class that we want to filter
    other_class = "person"    # other classes for showing
    detected_objects = []     # Empty List to store the detected objects of we consider

    for bbox in data.bounding_boxes:
        if bbox.class_id in [target_class, other_class]:  # Filter classes
            center = calculate_center(bbox)
            detected_objects.append({
                "class": bbox.class_id,
                "center": center,
                "probability": bbox.probability
            })

    # Print the results to the console
    for obj in detected_objects:
        rospy.loginfo(f"Detected {obj['class']} at {obj['center']} with probability {obj['probability']}")

    # For storing save the results to a CSV file
    with open("detected_objects.csv", "a") as csvfile:
        writer = csv.writer(csvfile)
        for obj in detected_objects:
            writer.writerow([obj["class"], *obj["center"], obj["probability"]])

# Main function
def main():
    rospy.init_node("yolo_3d_data_extraction", anonymous=True)

    # Subscribe to the bounding boxes topic
    rospy.Subscriber("/darknet_ros_3d/bounding_boxes", BoundingBox3DArray, bounding_boxes_callback)

    rospy.loginfo("YOLO 3D data extraction node started. Listening for bounding boxes...")
    rospy.spin()

if __name__ == "__main__":
    main()

#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion
import csv
from demo_yolo.msg import Data  
import os

class RobotNavigation():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Action client for move_base
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.loginfo("Connected to move_base server.")

        # Subscribe to detected objects
        rospy.Subscriber("/detected_objects", Data, self.object_detected_callback)
        self.detected_object = None  # Store detected object

    def object_detected_callback(self, msg):
        """ Callback function to process detected objects """
        rospy.loginfo(f"Received detected object: {msg.data}")
        self.detected_object = msg.data  # Update detected object

        # Move the robot if the detected object is "person"
        if self.detected_object == "person":
            rospy.loginfo("Person detected! Moving to room.")
            os.system('cvlc --play-and-exit --quiet /home/vladilena/piper/follow.wav')
          
            self.go_to_location("room")
            sys.exit()

    def go_to_location(self, position_name):
        x, y, theta = self.read_position(position_name)
        x, y, theta = float(x), float(y), float(theta)

        # Convert euler to quaternion
        q = quaternion_from_euler(0, 0, theta) 
        goal = MoveBaseGoal()        
        goal.target_pose.header.frame_id = 'map'        
        goal.target_pose.header.stamp = rospy.get_rostime()        
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))

        self.move_base.send_goal(goal)
        rospy.loginfo("Navigating to target location...")

        success = self.move_base.wait_for_result(rospy.Duration(60))
        state = self.move_base.get_state()        

        if success and state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Arrived at the target location.")
            return True
        else:
            self.move_base.cancel_goal()
            rospy.logwarn("Failed to reach target location.")
            return False

    def read_csv(self):        
        """ Reads the CSV file and returns a dictionary of locations """
        location_dict = {}        
        try:
            with open("/home/vladilena/catkin_ws/src/my_navigation/src/location.csv", "r") as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for row in csv_reader:
                    location_dict[row[0]] = [row[1], row[2], row[3]]
        except Exception as e:
            rospy.logerr(f"Error reading CSV file: {e}")

        return location_dict

    def read_position(self, position_name):        
        """ Fetches the coordinates for a given position name """
        dict_position = self.read_csv()        
        if position_name in dict_position:
            return dict_position[position_name]
        else:
            rospy.logwarn(f"Position {position_name} not found in CSV.")
            return [0, 0, 0]

    def shutdown(self):
        """ Stops the robot when shutting down """
        self.move_base.cancel_all_goals()
        rospy.loginfo("Stopping the robot.")

if __name__ == '__main__':
    try:
        RobotNavigation()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node terminated.")

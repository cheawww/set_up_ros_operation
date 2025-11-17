#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import subprocess
from geometry_msgs.msg import Twist
from rct_ahr_arm_controller.srv import GripperLevel, GoToArmPose
from rct_ahr_msgs.srv import NavToLocation
import os
import time
from demo_yolo.msg import BoundingBoxes


class ArmPoseState(smach.State):
    def __init__(self, pose_name):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.pose_name = pose_name
        self.pose_srv = rospy.ServiceProxy('/rct/arm/go_to_pose', GoToArmPose)

    def execute(self, ud):
        rospy.wait_for_service('/rct/arm/go_to_pose')
        try:
            rospy.loginfo(f" Moving arm to pose: {self.pose_name}")
            response = self.pose_srv(self.pose_name)
            return 'success' if response.success == False else 'failure'  
        except rospy.ServiceException as e:
            rospy.logerr(f"Arm pose service failed: {e}")
            return 'failure'


#Gripper Level
class GripperState(smach.State):
    def __init__(self, level):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.level = level
        self.gripper_srv = rospy.ServiceProxy('/rct/gripper/level', GripperLevel)

    def execute(self, ud):
        rospy.wait_for_service('/rct/gripper/level')
        try:
            rospy.loginfo(f" Setting gripper level to {self.level}")
            response = self.gripper_srv(self.level)
            return 'success' if response.success == False else 'failure'  
        except rospy.ServiceException as e:
            rospy.logerr(f"Gripper service failed: {e}")
            return 'failure'


#Move Forward
class WalkForwardState(smach.State):
    def __init__(self, duration=4.0, speed=0.05):
        smach.State.__init__(self, outcomes=['success'])
        self.duration = duration
        self.speed = speed
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, ud):
        rospy.loginfo(" 5 seconds")
        move_cmd = Twist()
        move_cmd.linear.x = self.speed

        rate = rospy.Rate(10)
        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < self.duration:
            self.pub.publish(move_cmd)
            rate.sleep()

        # Stop
        self.pub.publish(Twist())
        rospy.loginfo(" Done moving")
        return 'success'


#Human Tracker
class RunHumanTrackerState(smach.State):
    def __init__(self, script_path):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.script_path = script_path

    def execute(self, ud):
        rospy.loginfo(f" Launching human tracker: {self.script_path}")
        try:
            subprocess.Popen(['python3', self.script_path])
            rospy.loginfo("successfully")
            return 'success'
        except Exception as e:
            rospy.logerr(f" Failed : {e}")
            return 'failure'
        
class PlayAudio(smach.State):
    def __init__(self, filename):
        smach.State.__init__(self, outcomes=['done'])
        self.filename = filename

    def execute(self, ud):
        path = f"/home/vladilena/th_open_ws/src/your_smach/script/{self.filename}"
        rospy.loginfo(f" Playing audio: {path}")
        os.system(f"cvlc --play-and-exit --quiet {path}")
        return 'done'
    
class WaitState(smach.State):
    def __init__(self, duration=7):
        smach.State.__init__(self, outcomes=['waited'])
        self.duration = duration

    def execute(self, ud):
        rospy.loginfo(f" Waiting {self.duration} before next person")
        time.sleep(self.duration)
        return 'waited'
    
class CenterOnPerson(smach.State):
    def __init__(self, image_width=640, center_threshold=50):
        smach.State.__init__(self, outcomes=['centered', 'no_person'])

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/bounding_boxes', BoundingBoxes, self.bbox_callback)
        self.image_width = image_width
        self.center_threshold = center_threshold

        self.person_found = False
        self.centered = False
        self.last_seen_time = rospy.Time.now()

    def bbox_callback(self, msg):
        twist = Twist()
        self.person_found = False
        self.centered = False

        for box in msg.boxes:
            if box.class_name.lower() == "person":
                self.person_found = True
                self.last_seen_time = rospy.Time.now()

                bbox_center_x = box.x + box.w / 2
                image_center_x = self.image_width / 2
                error = bbox_center_x - image_center_x

                if abs(error) < self.center_threshold:
                    twist.angular.z = 0.0
                    self.centered = True
                else:
                    twist.angular.z = -0.002 * error 

                self.cmd_pub.publish(twist)
                return  

        self.cmd_pub.publish(Twist()) 

    def execute(self, userdata):
        rospy.loginfo("Centering")
        rate = rospy.Rate(10)
        timeout = 3.0

        while not rospy.is_shutdown():
            if self.centered:
                rospy.loginfo("centered")
                return 'centered'

            if (rospy.Time.now() - self.last_seen_time).to_sec() > timeout:
                rospy.logwarn("Person lost")
                return 'no_person'

            rate.sleep()

def main():
    rospy.init_node('smach_grab_flow')

    sm = smach.StateMachine(outcomes=['DONE'])

    with sm:

        smach.StateMachine.add('CENTER_ON_PERSON', CenterOnPerson(), 
                               transitions={'centered': 'POSE_READY',
                                            'no_person': 'POSE_READY'})
        smach.StateMachine.add("POSE_READY", ArmPoseState("ready"),
                               transitions={'success': 'GRIPPER_OPEN', 'failure': 'DONE'})

        smach.StateMachine.add("GRIPPER_OPEN", GripperState(2),
                               transitions={'success': 'GREET_1', 'failure': 'DONE'})
        
        smach.StateMachine.add("GREET_1", PlayAudio("luggage.wav"), transitions={'done': 'WAIT_2'})

        smach.StateMachine.add("WAIT_2", WaitState(5), transitions={'waited': 'GO_1'})

        smach.StateMachine.add("GO_1", WalkForwardState(), transitions={'success': 'GRIPPER_CLOSE'})

        


        smach.StateMachine.add("GRIPPER_CLOSE", GripperState(0),
                               transitions={'success': 'WAIT_4', 'failure': 'DONE'})
        smach.StateMachine.add("WAIT_4", WaitState(2), transitions={'waited': 'POSE_HOME'})

        smach.StateMachine.add("POSE_HOME", ArmPoseState("home"),
                               transitions={'success': 'WAIT_3', 'failure': 'DONE'})
        
        smach.StateMachine.add("WAIT_3", WaitState(3), transitions={'waited': 'GREET_2'})

        smach.StateMachine.add("GREET_2", PlayAudio("follow.wav"), transitions={'done': 'RUN_TRACKER'})
        

        smach.StateMachine.add("RUN_TRACKER", RunHumanTrackerState(
            "/home/vladilena/th_open_ws/src/your_package/script/human_tracker.py"),
            transitions={'success': 'DONE', 'failure': 'DONE'})

    try:
        outcome = sm.execute()
    except KeyboardInterrupt:
        rospy.logwarn(" KeyboardInterrupt received! Stopping...")
        RunHumanTrackerState.request_preempt()
        rospy.signal_shutdown("User requested shutdown with Ctrl+C")

if __name__ == '__main__':
    main()

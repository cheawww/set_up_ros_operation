#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import time
import json
import queue
import os
import subprocess
import sounddevice as sd
from vosk import Model, KaldiRecognizer
from geometry_msgs.msg import Twist
from demo_yolo.msg import BoundingBoxes, BoundingBox  
from rct_ahr_msgs.srv import NavToLocation
from rct_ahr_arm_controller.srv import GripperLevel, GoToArmPose

q = queue.Queue()


def audio_callback(indata, frames, time, status):
    if status:
        print(f"Audio status: {status}")
    q.put(bytes(indata))

def recognize_speech():
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "model")
        
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        model = Model(model_path)
        recognizer = KaldiRecognizer(model, 16000)

        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                               channels=1, callback=audio_callback):
            print("Speak")
            while True:
                data = q.get()
                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    text = result.get('text', '')
                    print(f"You said: {text}")
                    return text
    except Exception as e:
        print(f" Error in recognize_speech(): {e}")
        return ""
        
class RotateUntilPersonState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/bounding_boxes', BoundingBoxes, self.callback)  
        self.found = False

    def callback(self, msg):
        for box in msg.boxes: 
            if box.class_name == "person":  
                self.found = True

    def execute(self, ud):
        rospy.loginfo("find person")
        twist = Twist()
        twist.angular.z = 0.05 
        rate = rospy.Rate(10)
        self.found = False

        while not self.found and not rospy.is_shutdown():
            self.pub.publish(twist)
            rate.sleep()

        self.pub.publish(Twist())  
        return 'found'



class RotateUntilChairState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'])
        self.sub = rospy.Subscriber('/demo_yolo/BoundingBoxes', BoundingBoxes, self.bbox_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bounding_boxes = []
        self.detected_left = False

    def bbox_callback(self, msg):
        self.bounding_boxes = msg.bounding_boxes
        for box in self.bounding_boxes:
            if box.Class == "human":
                center_x = (box.xmin + box.xmax) / 2
                image_width = 640 
                if center_x < image_width / 2:
                    self.detected_left = True
                    break
                else:
                    self.detected_left = False

    def execute(self, userdata):
        rospy.loginfo("Looking for a person")
        rate = rospy.Rate(10)
        twist = Twist()
        while not rospy.is_shutdown():
            if self.detected_left:
                rospy.loginfo("Detected person")
                self.cmd_pub.publish(Twist()) 
                return 'found'

            # Keep rotating
            twist.angular.z = 0.5
            self.cmd_pub.publish(twist)
            rate.sleep()

        return 'not_found'

class PlayAudio(smach.State):
    def __init__(self, filename):
        smach.State.__init__(self, outcomes=['done'])
        self.filename = filename

    def execute(self, ud):
        path = f"/home/vladilena/th_open_ws/src/your_smach/script/{self.filename}"
        rospy.loginfo(f"Playing audio: {path}")
        os.system(f"cvlc --play-and-exit --quiet {path}")
        return 'done'

class GetSpeechInput(smach.State):
    def __init__(self, key):
        smach.State.__init__(self, outcomes=['received'], output_keys=[key])
        self.key = key

    def execute(self, ud):
        text = recognize_speech()
        rospy.loginfo(f"Heard: {text}")
        ud[self.key] = text
        return 'received'

class NavigateToState(smach.State):
    def __init__(self, location):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.location = location
        self.nav_srv = rospy.ServiceProxy('/rct/nav/nav_to_location', NavToLocation)

    def execute(self, ud):
        rospy.wait_for_service('/rct/nav/nav_to_location')
        try:
            self.nav_srv(self.location)
            return 'succeeded'  
        except rospy.ServiceException as e:
            rospy.logerr(f"Navigation service failed: {e}")
            return 'failed'  


class IntroducePerson(smach.State):
    def __init__(self, person_key, drink_key):
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=[person_key, drink_key])  

        self.person_key = person_key
        self.drink_key = drink_key

        # Piper config
        self.piper_model = "/home/vladilena/piper/en_GB-semaine-medium.onnx"
        self.piper_binary = "/home/vladilena/piper/piper/piper"
        self.audio_path = "/home/vladilena/th_open_ws/src/your_smach/script/"

    def speak(self, text, output_name):
        output_wav = os.path.join(self.audio_path, output_name)
        command = f'echo "{text}" | {self.piper_binary} --model {self.piper_model} --output_file {output_wav}'
        subprocess.run(command, shell=True, executable="/bin/bash", check=True)
        os.system(f"cvlc --play-and-exit --quiet {output_wav}")

    def execute(self, ud):
        person = ud[self.person_key]
        drink = ud[self.drink_key]

        rospy.loginfo(f"Introducing {person} and their favorite drink {drink}")

        os.system(f"cvlc --play-and-exit --quiet {self.audio_path}into_person.wav")

        self.speak(f"This is {person}", "say_person.wav")

        os.system(f"cvlc --play-and-exit --quiet {self.audio_path}into_drink.wav")

        self.speak(f"Their favorite drink is {drink}", "say_drink.wav")

        return 'success'


# Arm Pose
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
            return 'success' if response.success == False else 'failure'  # reverse logic
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
            return 'success' if response.success == False else 'failure'  # reverse logic
        except rospy.ServiceException as e:
            rospy.logerr(f"Gripper service failed: {e}")
            return 'failure'

class WaitState(smach.State):
    def __init__(self, duration=7):
        smach.State.__init__(self, outcomes=['waited'])
        self.duration = duration

    def execute(self, ud):
        rospy.loginfo(f"⏳ Waiting {self.duration} seconds before next person...")
        time.sleep(self.duration)
        return 'waited'
    
class UTurnState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, ud):
        rospy.loginfo(" U-turn ")
        
        twist = Twist()
        twist.angular.z = 0.5  
        rate = rospy.Rate(10)  
        
        start_time = rospy.get_time()
        duration = 3.14  
        
        while rospy.get_time() - start_time < duration:
            self.pub.publish(twist)
            rate.sleep()
        
        self.pub.publish(Twist())  
        rospy.loginfo("completed")
        return 'completed'

def main():
    rospy.init_node('receptionist_smach')

    sm = smach.StateMachine(outcomes=['END'])
    sm.userdata.person_1 = ''
    sm.userdata.drink_1 = ''
    sm.userdata.person_2 = ''
    sm.userdata.drink_2 = ''

    with sm:
        smach.StateMachine.add("WAIT_PERSON_1", RotateUntilPersonState(), transitions={'found': 'GREET_1', 'not_found': 'WAIT_5_MIN'})
        # Add the 5-second wait state after each stage
        smach.StateMachine.add("GREET_1", PlayAudio("hello_name.wav"), transitions={'done': 'ASK_NAME_1'})
        smach.StateMachine.add("ASK_NAME_1", GetSpeechInput("person_1"), transitions={'received': 'WAIT_2'})
        smach.StateMachine.add("WAIT_2", WaitState(5), transitions={'waited': 'ASK_DRINK_PROMPT_1'})
        smach.StateMachine.add("ASK_DRINK_PROMPT_1", PlayAudio("fav_drink.wav"), transitions={'done': 'GET_DRINK_1'})
        smach.StateMachine.add("GET_DRINK_1", GetSpeechInput("drink_1"), transitions={'received': 'WAIT_4'})
        smach.StateMachine.add("WAIT_4", WaitState(5), transitions={'waited': 'FOLLOW_1'})
        smach.StateMachine.add("FOLLOW_1", PlayAudio("go_host.wav"), transitions={'done': 'WAIT_5'})
        smach.StateMachine.add("WAIT_5", WaitState(5), transitions={'waited': 'GO_TO_HOST_1'})
        smach.StateMachine.add("GO_TO_HOST_1", NavigateToState("host2"), transitions={'succeeded': 'WAIT_6', 'failed': 'WAIT_6'})
        smach.StateMachine.add("WAIT_6", WaitState(30), transitions={'waited': 'FIND_PERSON_2'})
        smach.StateMachine.add("FIND_PERSON_2", RotateUntilPersonState(), transitions={'found': 'WAIT_7', 'not_found': 'WAIT_5_MIN'})
        smach.StateMachine.add("WAIT_7", WaitState(5), transitions={'waited': 'INTRODUCE_1'})
        smach.StateMachine.add("INTRODUCE_1", IntroducePerson('person_1', 'drink_1'),
                            transitions={'success': 'WAIT_8'})
        smach.StateMachine.add("WAIT_8", WaitState(5), transitions={'waited': 'U_TURN_1'})
        smach.StateMachine.add("U_TURN_1", UTurnState(), transitions={'completed': 'WAIT_23'})
        smach.StateMachine.add("WAIT_23", WaitState(5), transitions={'waited': 'GREET_15'})
        smach.StateMachine.add("GREET_15", PlayAudio("host_name.wav"), transitions={'done': 'GREET_16'})
        smach.StateMachine.add("GREET_16", PlayAudio("host_drink.wav"), transitions={'done': 'WAIT_25'})
        smach.StateMachine.add("WAIT_25", WaitState(5), transitions={'waited': 'FIND_CHAIR_1'})
        smach.StateMachine.add("FIND_CHAIR_1", RotateUntilChairState(), transitions={'found': 'WAIT_9', 'not_found': 'WAIT_5_MIN'})
        smach.StateMachine.add("WAIT_9", WaitState(5), transitions={'waited': 'SITE_THERE_1'})
        smach.StateMachine.add("SITE_THERE_1", PlayAudio("site_there.wav"), transitions={'done': 'WAIT_10'})
        smach.StateMachine.add("WAIT_10", WaitState(5), transitions={'waited': 'GO_TO_GUESS_1'})
        smach.StateMachine.add("POSE_READY", ArmPoseState("there"), transitions={'success': 'WAIT_5_MIN', 'failure': 'END'})
        smach.StateMachine.add("WAIT_5_MIN", WaitState(), transitions={'waited': 'POSE_2'})
        smach.StateMachine.add("POSE_2", ArmPoseState("home"), transitions={'success': 'WAIT_11', 'failure': 'END'})
        smach.StateMachine.add("WAIT_11", WaitState(5), transitions={'waited': 'GO_TO_GUESS_1'})
        smach.StateMachine.add("GO_TO_GUESS_1", NavigateToState("guess"), transitions={'succeeded': 'WAIT_12', 'failed': 'WAIT_12'})

        smach.StateMachine.add("WAIT_12", WaitState(5), transitions={'waited': 'WAIT_PERSON_2'})

        smach.StateMachine.add("WAIT_PERSON_2", RotateUntilPersonState(), transitions={'found': 'WAIT_13', 'not_found': 'WAIT_5_MIN'})
        smach.StateMachine.add("WAIT_13", WaitState(5), transitions={'waited': 'GREET_2'})
        smach.StateMachine.add("GREET_2", PlayAudio("hello_name.wav"), transitions={'done': 'ASK_NAME_2'})
        smach.StateMachine.add("ASK_NAME_2", GetSpeechInput("person_2"), transitions={'received': 'WAIT_15'})
        smach.StateMachine.add("WAIT_15", WaitState(5), transitions={'waited': 'ASK_DRINK_PROMPT_2'})
        smach.StateMachine.add("ASK_DRINK_PROMPT_2", PlayAudio("fav_drink.wav"), transitions={'done': 'GET_DRINK_2'})
        smach.StateMachine.add("GET_DRINK_2", GetSpeechInput("drink_2"), transitions={'received': 'WAIT_17'})
        smach.StateMachine.add("WAIT_17", WaitState(5), transitions={'waited': 'GO_TO_HOST_2'})
        smach.StateMachine.add("GO_TO_HOST_2", NavigateToState("host2"), transitions={'succeeded': 'WAIT_18', 'failed': 'WAIT_18'})
        smach.StateMachine.add("WAIT_18", WaitState(5), transitions={'waited': 'FIND_PERSON_3'})
        smach.StateMachine.add("FIND_PERSON_3", RotateUntilPersonState(), transitions={'found': 'WAIT_19', 'not_found': 'WAIT_5_MIN'})
        smach.StateMachine.add("WAIT_19", WaitState(5), transitions={'waited': 'INTRODUCE_2'})
        smach.StateMachine.add("INTRODUCE_2", IntroducePerson('person_2', 'drink_2'),
                            transitions={'success': 'WAIT_20'})
        smach.StateMachine.add("WAIT_20", WaitState(5), transitions={'waited': 'U_TURN_2'})
        smach.StateMachine.add("U_TURN_2", UTurnState(), transitions={'completed': 'WAIT_26'})
        smach.StateMachine.add("WAIT_26", WaitState(5), transitions={'waited': 'GREET_18'})
        smach.StateMachine.add("GREET_18", PlayAudio("host_name.wav"), transitions={'done': 'GREET_17'})
        smach.StateMachine.add("GREET_17", PlayAudio("host_drink.wav"), transitions={'done': 'WAIT_27'})
        smach.StateMachine.add("WAIT_27", WaitState(5), transitions={'waited': 'FIND_CHAIR_2'})        
        smach.StateMachine.add("FIND_CHAIR_2", RotateUntilChairState(), transitions={'found': 'WAIT_21', 'not_found': 'WAIT_7_MIN'})
        smach.StateMachine.add("WAIT_21", WaitState(5), transitions={'waited': 'GO_TO_GUESS_2'})
        smach.StateMachine.add("GO_TO_GUESS_2", NavigateToState("guess"),  transitions={'succeeded': 'WAIT_22', 'failed': 'WAIT_22'})
        smach.StateMachine.add("WAIT_22", WaitState(5), transitions={'waited': 'WAIT_7_MIN'})
        smach.StateMachine.add("WAIT_7_MIN", WaitState(), transitions={'waited': 'END'})

    try:
        outcome = sm.execute()
    except KeyboardInterrupt:
        rospy.logwarn(" KeyboardInterrupt received! Stopping...")

        rospy.signal_shutdown("User requested shutdown with Ctrl+C")

if __name__ == '__main__':
    main()

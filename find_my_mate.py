#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import subprocess
import os
import queue
import json
import sounddevice as sd
from vosk import Model, KaldiRecognizer
from std_msgs.msg import String
from demo_yolo.msg import Data
from geometry_msgs.msg import Twist

PIPER_MODEL = "/home/vladilena/piper/en_GB-semaine-medium.onnx"
PIPER_BINARY = "/home/vladilena/piper/piper/piper"

vosk_model = Model("model")
recognizer = KaldiRecognizer(vosk_model, 16000)
audio_q = queue.Queue()

def audio_callback(indata, frames, time, status):
    audio_q.put(bytes(indata))

def speak(text):
    output_file = "/tmp/speak.wav"
    command = f'echo "{text}" | {PIPER_BINARY} --model {PIPER_MODEL} --output_file {output_file}'
    subprocess.run(command, shell=True, executable="/bin/bash")
    os.system(f"cvlc --play-and-exit --quiet {output_file}")

class DetectHuman(smach.State):
    def __init__(self, outcomes=['found', 'not_found']):
        smach.State.__init__(self, outcomes)
        self.sub = rospy.Subscriber('/detected_objects', Data, self.callback)
        self.detected = False

    def callback(self, msg):
        if "person" in msg.data:
            self.detected = True

    def execute(self, ud):
        rospy.loginfo("Looking")
        self.detected = False
        rospy.sleep(2.0) 
        return 'found' if self.detected else 'not_found'

class AskName(smach.State):
    def __init__(self, outcomes=['got_name'], output_keys=['person_name']):
        smach.State.__init__(self, outcomes, output_keys=output_keys)

    def execute(self, ud):
        rospy.loginfo("Asking for name")
        speak("Hello. What is your name?")

        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                               channels=1, callback=audio_callback):
            rospy.loginfo("Listening for name")
            name = ""
            while not name:
                data = audio_q.get()
                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    name = result['text']

        rospy.loginfo(f" Name received: {name}")
        ud.person_name = name
        return 'got_name'

class RotateToSearch(smach.State):
    def __init__(self, outcomes=['rotated']):
        smach.State.__init__(self, outcomes)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, ud):
        rospy.loginfo("Rotating to search for another person")
        twist = Twist()
        twist.angular.z = 0.5
        start_time = rospy.Time.now()
        r = rospy.Rate(10)
        while (rospy.Time.now() - start_time).to_sec() < 3:
            self.pub.publish(twist)
            r.sleep()
        twist.angular.z = 0.0
        self.pub.publish(twist)
        return 'rotated'

class GreetPersons(smach.State):
    def __init__(self, outcomes=['done'], input_keys=['person1', 'person2']):
        smach.State.__init__(self, outcomes, input_keys=input_keys)

    def execute(self, ud):
        message = f"Nice to meet you {ud.person1} and {ud.person2}. Let's continue."
        rospy.loginfo("Greeting people")
        speak(message)
        return 'done'

# --- Main ---
def main():
    rospy.init_node("smach_name_capture_robot")

    sm = smach.StateMachine(outcomes=['DONE'])
    sm.userdata.person1 = ""
    sm.userdata.person2 = ""

    with sm:
        smach.StateMachine.add("DETECT1", DetectHuman(), transitions={'found': 'ASK1', 'not_found': 'DETECT1'})
        smach.StateMachine.add("ASK1", AskName(), transitions={'got_name': 'ROTATE'}, remapping={'person_name': 'person1'})
        smach.StateMachine.add("ROTATE", RotateToSearch(), transitions={'rotated': 'DETECT2'})
        smach.StateMachine.add("DETECT2", DetectHuman(), transitions={'found': 'ASK2', 'not_found': 'DETECT2'})
        smach.StateMachine.add("ASK2", AskName(), transitions={'got_name': 'GREET'}, remapping={'person_name': 'person2'})
        smach.StateMachine.add("GREET", GreetPersons(), transitions={'done': 'DONE'})

    outcome = sm.execute()

if __name__ == "__main__":
    main()

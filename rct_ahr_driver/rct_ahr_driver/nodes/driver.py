#!/usr/bin/env python3

import serial
import time

import rospy
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist, TransformStamped

FREQ_SEND_CMD = 25 # Hz
FREQ_GET_DATA = 10 # Hz
MAX_VEL_X = 0.25 # m/s
MAX_VEL_TH = 0.5 # m/s
WAITING_FOR_ODOM_TIMEOUT = 0.1 #s

class RCTAtHomeRobotDriver():
    def __init__(self):

        try:
            self.robot = serial.Serial("/dev/rct_ahr_robot", 115200, timeout = 1, parity = serial.PARITY_EVEN, dsrdtr = True)
        except serial.serialutil.SerialException:
            rospy.logerr("Please connect robot to your computer.")
            exit()
        
        self.cmd_vel_subscription = rospy.Subscriber("/smooth_cmd_vel", Twist, self.get_cmd_vel_callback)
        self.reset_odom_service = rospy.Service("/reset_odom", Empty, self.reset_odom_callback)
        self.tf_broadcaster = TransformBroadcaster()

        timer_send_cmd = 1 / FREQ_SEND_CMD  # seconds
        timer_get_data = 1 / FREQ_GET_DATA  # seconds

        self.cmd_vel_x = 0.0
        self.cmd_vel_th = 0.0
        self.odom = [0.0, 0.0, 0.0] # x y th
        self.data = ""
        self.cmd_timeout_count = 0

    def get_cmd_vel_callback(self, msg):
        cmd_vel_x = msg.linear.x
        cmd_vel_th = msg.angular.z
        # rospy.loginfo(f"cmd_vel_x : {cmd_vel_x}, cmd_vel_th : {cmd_vel_th}")
        self.cmd_vel_x = cmd_vel_x
        self.cmd_vel_th = cmd_vel_th

    def reset_odom_callback(self, data):
        req = "R#".encode()
        self.robot.write(req)
        return EmptyResponse

    def get_odom_from_serial(self):
        
        req = "O#".encode()
        self.robot.write(req)
        time.sleep(0.02)

        if (self.robot.in_waiting > 0):
            # Message => Ox:+000.00,y:+000.00,th:+000.00#
            data = self.robot.read(1)
            data = data.decode()
            if data == "O":
                data = self.robot.read(31)
                # print(data)
                self.robot.reset_input_buffer()
                data = data.decode()
                if data[30] == "#":
                    data = data[:30]
                    data = data.split(",")
                    for i in range(3): # x y th
                        tmp = data[i].split(":")
                        self.odom[i] = float(tmp[1])
                    
                    t = TransformStamped()

                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_footprint'
                    t.transform.translation.x = self.odom[0]
                    t.transform.translation.y = self.odom[1]
                    t.transform.translation.z = 0.0
                    
                    q = quaternion_from_euler(0, 0, self.odom[2])
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]

                    # Send the transformation
                    self.tf_broadcaster.sendTransform(t)

                    # rospy.loginfo(f"t: {t}")
                    # rospy.loginfo("Send transform")

                    # break
                else:
                    self.robot.reset_input_buffer()
                    rospy.logerr(f"Can't get data : {data}")
                    # continue
            else:
                self.robot.reset_input_buffer()
                rospy.logerr(f"Can't understand start : {data}")
                # continue
        else:
            pass
            # continue

    def send_cmd_to_serial(self):
        cmd_vel_x = self.cmd_vel_x
        cmd_vel_th = self.cmd_vel_th
        stop = False
        cmd = ""
        if cmd_vel_x == 0 and cmd_vel_th == 0:
            cmd = "S"
            stop = True
        elif cmd_vel_x >= 0:
            cmd += "F"
        elif cmd_vel_x < 0:
            cmd += "B"
        if not stop: 
            # Transform cmd_vel_x to serial template
            cmd += str(int(abs(cmd_vel_x) % 10))
            cmd += str(int((abs(cmd_vel_x) * 10) % 10))
            cmd += str(int((abs(cmd_vel_x) * 100) % 10))

            cmd += "|"

            if cmd_vel_th >= 0:
                cmd += "+"
            elif cmd_vel_th < 0:
                cmd += "-"
            else:
                pass

            # Transform cmd_vel_x to serial template
            cmd += str(int(abs(cmd_vel_th) % 10))
            cmd += str(int((abs(cmd_vel_th) * 10) % 10))
            cmd += str(int((abs(cmd_vel_th) * 100) % 10))
        cmd += "#"
        # print(cmd.encode())
        self.robot.write(cmd.encode())

def main(args = None):
    rospy.init_node("rct_ahr_driver_node")

    rct_ahr_driver_node = RCTAtHomeRobotDriver()

    rate = rospy.Rate(30) # 30 Hz

    while not rospy.is_shutdown():
        rct_ahr_driver_node.send_cmd_to_serial()
        rct_ahr_driver_node.get_odom_from_serial()
        rate.sleep()
        pass

if __name__ == "__main__":
    main()
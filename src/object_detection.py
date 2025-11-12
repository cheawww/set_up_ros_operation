#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import rospkg
from demo_yolo.msg import BoundingBox, BoundingBoxes, Data  # Make sure these are defined in your package

path = rospkg.RosPack().get_path("demo_yolo")
os.chdir(path)

class ObjectDetection:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("object_detect", anonymous=True)
        
        # Subscriber
        rospy.Subscriber("/usb_cam/image_raw", Image, self.update_frame_callback)
        rospy.wait_for_message("/usb_cam/image_raw", Image)

        # Publishers
        self.name_pub = rospy.Publisher("/detected_objects", Data, queue_size=10)
        self.box_pub = rospy.Publisher("/bounding_boxes", BoundingBoxes, queue_size=10)

        self.image = None  # Latest image frameQ

    def update_frame_callback(self, data):
        """Callback to update the latest camera frame."""
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    def publish_detection(self, detected_objects):
        """Publish detected object names as a string."""
        msg = Data()
        msg.data = ", ".join(detected_objects)
        self.name_pub.publish(msg)

    def publish_bounding_boxes(self, class_ids, boxes, classes):
        """Publish bounding box info with class names."""
        msg = BoundingBoxes()
        for i in range(len(class_ids)):
            box = BoundingBox()
            box.class_name = classes[class_ids[i]]
            box.x = boxes[i][0]
            box.y = boxes[i][1]
            box.w = boxes[i][2]
            box.h = boxes[i][3]
            msg.boxes.append(box)
        self.box_pub.publish(msg)

    def main(self):
        """Runs YOLO object detection and publishes both object names and bounding boxes."""
        net = cv2.dnn.readNet("weight/yolov3.weights", "cfg/yolov3.cfg")

        with open("cfg/coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]

        output_layers = net.getUnconnectedOutLayersNames()
        colors = np.random.uniform(0, 255, size=(len(classes), 3))

        while not rospy.is_shutdown():
            if self.image is None:
                continue

            frame = self.image.copy()
            height, width, _ = frame.shape
            blob = cv2.dnn.blobFromImage(frame, scalefactor=0.00392, size=(320, 320), mean=(0, 0, 0), swapRB=True, crop=False)
            net.setInput(blob)
            outputs = net.forward(output_layers)

            boxes, confs, class_ids = [], [], []
            detected_objects = []

            for output in outputs:
                for detect in output:
                    scores = detect[5:]
                    class_id = np.argmax(scores)
                    conf = scores[class_id]
                    if conf > 0.3:
                        center_x = int(detect[0] * width)
                        center_y = int(detect[1] * height)
                        w = int(detect[2] * width)
                        h = int(detect[3] * height)
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)

                        boxes.append([x, y, w, h])
                        confs.append(float(conf))
                        class_ids.append(class_id)

            indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
            font = cv2.FONT_HERSHEY_PLAIN

            filtered_class_ids = []
            filtered_boxes = []

            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    detected_objects.append(label)

                    filtered_class_ids.append(class_ids[i])
                    filtered_boxes.append([x, y, w, h])

                    color = colors[class_ids[i]]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(frame, label, (x, y - 5), font, 1, color, 1)

            # Publish detected names
            if detected_objects:
                self.publish_detection(detected_objects)

            # Publish bounding boxes
            if filtered_class_ids:
                self.publish_bounding_boxes(filtered_class_ids, filtered_boxes, classes)

            cv2.imshow("YOLO Detection", frame)
            if cv2.waitKey(1) == 27:
                break

if __name__ == "__main__":
    obj = ObjectDetection()
    obj.main()

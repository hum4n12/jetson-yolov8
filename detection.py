from ultralytics import YOLO
import cv2
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from typing import List
from statistics import median

class Detection:
    def __init__(self) -> None:
        self.model = YOLO("yolov8m.pt") #pretrained on COCO dataset
        self.model_full = YOLO("yolov8m.pt")
        self.probability_threshold = 0.6
        self.detected_objects = []
        self.id = 1
    
    def get_detected_objects(self):
        return self.detected_objects

    def create_pose(self) -> Marker:
        initpose = Marker()
        initpose.header.stamp = rospy.get_rostime()
        initpose.header.frame_id = "ignore"
        initpose.pose.orientation.w = 0.0
        initpose.pose.orientation.x = 0.0
        initpose.pose.orientation.y = 0.0
        initpose.pose.orientation.z = 0.0
        initpose.id = self.id
        self.id += 1
        return initpose

    def detect(self, image, depth, classes = []) -> MarkerArray:
        cv_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        print(classes)
        results = self.model_full(cv_image) if len(classes) == 0 else self.model(cv_image, classes=classes)
        positions = []
        self.detected_objects = []
        for box in results[0].boxes:
            if not box:
                break
            
            conf = float(box.conf[0].item())
            if conf < self.probability_threshold:
                break
            
            item_class = int(box.cls[0].item())
            print("item_class: ", item_class)
            class_name = results[0].names[item_class]
            print("class_names: ", class_name)
            pose = self.create_pose()
            pose.header.frame_id = class_name
            coords = [item.item() for item in box.xyxy[0]]
            center_x = (coords[2] - coords[0]) / 2 + coords[0]
            center_y = (coords[3] - coords[1]) / 2 + coords[1]
            pose.pose.position.x = center_x
            pose.pose.position.y = center_y

            sqr = []
            for j in range(10):
                for i in range(10):
                    if center_y+i-5< 720 and center_x+i-5 < 1280:
                        # print(float(depth[int(center_y+j-5)][int(center_x+i-5)]))
                        if float(depth[int((center_y+j-5))][int((center_x+i-5))]) != 0:
                            # pose.pose.position.z = float(depth[int(center_y+j-2)][int(center_x+i-2)])
                            sqr.append(float(depth[int(center_y+j-5)][int(center_x+i-5)]))
                            break
                if pose.pose.position.z != 0:
                    break
            if len(sqr) != 0:
                pose.pose.position.z = median(sqr)
            else:
                pose.pose.position.z = 0

            print(f"Position: X: {center_x} , Y: {center_y}, Z: {pose.pose.position.z}")
            positions.append(pose)
            self.detected_objects.append({
                "label": class_name,
                "box": coords
            })

        return positions
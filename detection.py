from ultralytics import YOLO
import cv2
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from typing import List

class Detection:
    def __init__(self) -> None:
        self.model = YOLO("yolov8m.pt") #pretrained on COCO dataset
        self.model_full = YOLO("yolov8m.pt")
        self.probability_threshold = 0.6
        self.image = None

    def get_image(self):
        return self.image

    def create_pose(self) -> PoseWithCovarianceStamped:
        initpose = PoseWithCovarianceStamped()
        initpose.header.stamp = rospy.get_rostime()
        initpose.header.frame_id = "ignore"
        initpose.pose.pose.orientation.w = 0.0
        initpose.pose.pose.orientation.x = 0.0
        initpose.pose.pose.orientation.y = 0.0
        initpose.pose.pose.orientation.z = 0.0
        return initpose

    def detect(self, image, classes = []) -> List[PoseWithCovarianceStamped]:
        cv_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        print(classes)
        results = self.model_full(cv_image) if len(classes) == 0 else self.model(cv_image, classes=classes)
        positions = []
        self.image = cv_image
        for box in results[0].boxes:
            if not box:
                break
            
            conf = float(box.conf[0].item())
            if conf < self.probability_threshold:
                break
            
            item_class = int(box.cls[0].item())
            class_name = results[0].names[item_class]
            pose = self.create_pose()
            pose.header.frame_id = class_name
            coords = [item.item() for item in box.xyxy[0]]
            center_x = (coords[2] - coords[0]) / 2 + coords[0]
            center_y = (coords[3] - coords[1]) / 2 + coords[1]
            pose.pose.pose.position.x = center_x
            pose.pose.pose.position.y = center_y
            positions.append(pose)
            
        self.image = results[0].plot()

        return positions

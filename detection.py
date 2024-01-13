from ultralytics import YOLO
import cv2
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from typing import List
from pprint import pprint
from statistics import median
class Detection:
    def __init__(self) -> None:
        self.model = YOLO("yolov8m.pt") # pretrained on COCO dataset
        self.probability_threshold = 0.8
        self.image = None
        self.id = 1

    def get_image(self):
        return self.image

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

    def detect(self, image, depth) -> MarkerArray:
        cv_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.model(cv_image)  # predict on an image
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
            pose.pose.position.x = center_x 
            pose.pose.position.x = center_y
            print(f"{class_name}")
            # pprint(depth[int(center_y-1)])
            sqr = []
            for j in range(10):
                for i in range(10):
                    if center_y+i-5< 720 and center_x+i-5 < 1280:
                        # print(float(depth[int(center_y+j-5)][int(center_x+i-5)]))
                        if float(depth[int((center_y+j-5))][int((center_x+i-5))]) != 0:
                            # pose.pose.position.z = float(depth[int(center_y+j-2)][int(center_x+i-2)])
                            sqr.append(float(depth[int(center_y+j-5)][int(center_x+i-5)]))
                #             break
                # if pose.pose.position.z != 0:
                #     break
            if len(sqr) != 0:
                pose.pose.position.z = median(sqr)
            else:
                pose.pose.position.z = 0 
            print(f"Position: X: {center_x} , Y: {center_y}, Z: {pose.pose.position.z}")
            positions.append(pose)
            self.draw(int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3]), class_name)

        return positions

    """
    def detect(self, image) -> MarkerArray:
        cv_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.model(cv_image)  # predict on an image
        # positions = []
        positions = MarkerArray()
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
            pose.ns = class_name
            coords = [item.item() for item in box.xyxy[0]]
            # środek bounding box'a
            center_x = (coords[2] - coords[0]) / 2 + coords[0]
            center_y = (coords[3] - coords[1]) / 2 + coords[1]
            pose.pose.position.x = center_x
            pose.pose.position.y = center_y
            positions.markers.append(pose)
            self.draw(int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3]), class_name)

        return positions
    """
    def draw(self, x1, y1, x2, y2, label):
        cv2.rectangle(self.image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 1
        text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]
        text_position = ((x1 + x2 - text_size[0]) // 2, y1 - 10)
        cv2.putText(self.image, label, text_position, font, font_scale, (0, 0, 255), font_thickness, cv2.LINE_AA)

import rospy
from ultralytics.utils.plotting import Annotator
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from detection import Detection
from geometry_msgs.msg import PoseWithCovarianceStamped
from typing import List
import json
import cv2

VIDEO_TOPIC='/camera/image_raw'
DEPTH_TOPIC='/camera/depth_raw'
COORDS_TOPIC='/coords'
ITEM_TOPIC='/item'
IMAGE_TOPIC='/image'

class Subscriber:
    def __init__(self):
        rospy.init_node('video_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.detector = Detection()
        self.publisher = rospy.Publisher(COORDS_TOPIC, PoseWithCovarianceStamped, queue_size=10)
        self.image_publisher = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=10)
        self.image_sub = rospy.Subscriber(VIDEO_TOPIC, Image, self.image_callback)
        self.item_sub = rospy.Subscriber(ITEM_TOPIC, String, self.items_callback)
        self.items = []
        self.annotator = None
        self.counter = 0

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_data(image)
        except CvBridgeError as e:
            print(e)
            return

    def process_data(self, image):
        if self.counter == 0:
            data: List[PoseWithCovarianceStamped] = self.detector.detect(image, self.items)
            # img = self.detector.get_image()
            for pose in data:
                self.publisher.publish(pose)
        
        self.counter += 1
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        annotator = Annotator(img)
        for coord in self.detector.get_image_coords():
            annotator.box_label(box = coord["box"], label = coord["label"])

        img = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.image_publisher.publish(img)

        if self.counter >= 12:
            self.counter = 0

    def items_callback(self, items):
        self.items = json.loads(items.data)



def main():
    subscriber = Subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
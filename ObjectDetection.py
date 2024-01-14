import rospy
from ultralytics.utils.plotting import Annotator
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from detection import Detection
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
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
        self.publisher = rospy.Publisher(COORDS_TOPIC, MarkerArray, queue_size=20)
        self.image_publisher = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=10)
        self.image_sub = rospy.Subscriber(VIDEO_TOPIC, Image, self.image_callback)
        self.item_sub = rospy.Subscriber(ITEM_TOPIC, String, self.items_callback)
        self.depth_sub = rospy.Subscriber(DEPTH_TOPIC, Image, self.depth_callback)
        self.image = None
        self.depth = 1
        self.items = []
        self.counter = 0

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.image is None:
                self.image = img
            
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            annotator = Annotator(img)
            for coord in self.detector.get_detected_objects():
                annotator.box_label(box = coord["box"], label = coord["label"])

            img = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
            self.image_publisher.publish(img)

        except CvBridgeError as e:
            print(e)
            return
        self.process_data()
        
    def depth_callback(self, msg):
        try:
            if self.depth is None:
                self.depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                # cv2.normalize(self.depth,self.depth,0,255,cv2.NORM_MINMAX)
                #min is 35cm
                #max is 12m = 1200cm
        except CvBridgeError as e:
            print(e)
            return
        self.process_data()

    def process_data(self):
        if self.image is None:
            return

        if self.counter == 0:
            data: List[PoseWithCovarianceStamped] = self.detector.detect(self.image, self.depth, self.items)
            self.publisher.publish(data)

        self.counter += 1
        if self.counter >= 20:
            self.counter = 0
        self.image = None
        self.depth = None

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
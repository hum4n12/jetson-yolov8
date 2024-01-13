import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from detection import Detection
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from typing import List

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
        self.depth_sub = rospy.Subscriber(DEPTH_TOPIC, Image, self.depth_callback)
        self.image = None
        self.depth = None

    def image_callback(self, msg):
        try:
            if self.image is None:
                self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
        # print(f"Depth: {self.depth} size: {len(self.depth)} size2: {len(self.depth[0])}")
        data: List[PoseWithCovarianceStamped] = self.detector.detect(self.image, self.depth)
        #data: MarkerArray = self.detector.detect(self.image)

        img = self.detector.get_image()
        self.image = None
        self.depth = None

        # cv2.imshow("img", img);
        # cv2.waitKey(1)

        img = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.image_publisher.publish(img)
        self.publisher.publish(data)


def main():
    subscriber = Subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

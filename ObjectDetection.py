import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from detection import Detection
from geometry_msgs.msg import PoseWithCovarianceStamped
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
        self.publisher = rospy.Publisher(COORDS_TOPIC, PoseWithCovarianceStamped, queue_size=10)
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
                self.depth = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        self.process_data()

    def process_data(self):
        if self.image is None or self.depth is  None:
            return
        
        data: List[PoseWithCovarianceStamped] = self.detector.detect(self.image, self.depth)
        img = self.detector.get_image()
        self.image = None
        self.depth = None
        img = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.image_publisher.publish(img)
        for pose in data:
            self.publisher.publish(pose)


def main():
    subscriber = Subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
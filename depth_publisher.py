import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

VIDEO_PATH = '../Downloads/zielu/v2/depth.mp4'

class DepthPublisher:
    def __init__(self, topic_name):
        rospy.init_node('depth_publisher', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(topic_name, Image, queue_size=10)

        self.video_capture = cv2.VideoCapture(VIDEO_PATH)
        # self.video_capture.set(cv2.CAP_PROP_FORMAT, cv2.CV_16F)

    def publish_frames(self):
        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            print("publishing")
            ret, frame = self.video_capture.read()
            if not ret:
                self.video_capture.release()
                self.video_capture = cv2.VideoCapture(VIDEO_PATH)
                continue

            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
            rate.sleep()

def main():
    topic_name = '/camera/depth_raw'

    depth_publisher = DepthPublisher(topic_name)
    try:
        depth_publisher.publish_frames()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
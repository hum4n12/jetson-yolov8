import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# VIDEO_PATH = '../Downloads/zielu/v2/left.mp4'
COLOR_VIDEO_PATH = '../Downloads/oakd/data3.mp4'
LEFT_VIDEO_PATH  = '../Downloads/oakd/left.mp4'
RIGHT_VIDEO_PATH = '../Downloads/oakd/right.mp4'


class VideoPublisher:
    def __init__(self, topic_name):
        rospy.init_node('video_publisher', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub_color = rospy.Publisher(topic_name, Image, queue_size=10)

        self.image_pub_left = rospy.Publisher('/camera/left', Image, queue_size=5)
        self.image_pub_right = rospy.Publisher('/camera/right', Image, queue_size=5)

        self.video_capture = cv2.VideoCapture(COLOR_VIDEO_PATH)
        self.video_capture_l = cv2.VideoCapture(LEFT_VIDEO_PATH)
        self.video_capture_r = cv2.VideoCapture(RIGHT_VIDEO_PATH)


    def publish_frames(self):
        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            print("publishing")
            ret1, frame_color = self.video_capture.read()
            ret2, frame_left = self.video_capture_l.read()
            ret3, frame_right = self.video_capture_r.read()

            if not ret1 or not ret2 or not ret3:
                self.video_capture.release()
                self.video_capture = cv2.VideoCapture(COLOR_VIDEO_PATH)

                self.video_capture_l.release()
                self.video_capture_l = cv2.VideoCapture(LEFT_VIDEO_PATH)

                self.video_capture_r.release()
                self.video_capture_r = cv2.VideoCapture(RIGHT_VIDEO_PATH)
                continue

            if ret1 and ret2 and ret3:
                ros_image = self.bridge.cv2_to_imgmsg(frame_color, "rgb8")
                self.image_pub_color.publish(ros_image)

                ros_image_left = self.bridge.cv2_to_imgmsg(frame_left, "rgb8")
                self.image_pub_left.publish(ros_image_left)

                ros_image_right = self.bridge.cv2_to_imgmsg(frame_right, "rgb8")
                self.image_pub_right.publish(ros_image_right)
            rate.sleep()

def main():
    topic_name = '/camera/image_raw'

    video_publisher = VideoPublisher(topic_name)
    try:
        video_publisher.publish_frames()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
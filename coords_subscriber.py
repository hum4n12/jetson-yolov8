import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

COORDS_TOPIC='/coords'

class VideoSubscriber:
    def __init__(self):
        rospy.init_node('coord_subscriber', anonymous=True)
        self.depth_sub = rospy.Subscriber(COORDS_TOPIC, PoseWithCovarianceStamped, self.image_callback)

    def image_callback(self, msg):
        if msg.header.frame_id == "ignore":
            return
        print(msg)

def main():
    video_subscriber = VideoSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
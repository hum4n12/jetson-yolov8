import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai as dai
VIDEO_PATH = './data3.mp4'

class VideoPublisher:
    def __init__(self, topic_name):
        rospy.init_node('video_publisher', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
	# Create pipeline
        self.pipeline = dai.Pipeline()
        self.image_pub_color = rospy.Publisher(topic_name, Image, queue_size=10)
        self.image_pub_depth = rospy.Publisher('/camera/depth_raw', Image, queue_size=5)
        self.image_pub_left = rospy.Publisher('/camera/left', Image, queue_size=5)
        self.image_pub_right = rospy.Publisher('/camera/right', Image, queue_size=5)
        # Closer-in minimum depth, disparity range is doubled (from 95 to 190):
        extended_disparity = False
        # Better accuracy for longer distance, fractional disparity 32-levels:
        subpixel = False
        # Better handling for occlusions:
        lr_check = True

	# Define source and output
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        xoutRight = self.pipeline.create(dai.node.XLinkOut)
        xoutRight.setStreamName("right")

        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        xoutLeft = self.pipeline.create(dai.node.XLinkOut)
        xoutLeft.setStreamName("left")

        colorCamera = self.pipeline.create(dai.node.ColorCamera)
        xoutColor = self.pipeline.create(dai.node.XLinkOut)
        xoutColor.setStreamName("video")

        depth = self.pipeline.create(dai.node.StereoDepth)
        xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")



	# Properties
        monoRight.setCamera("right")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        monoLeft.setCamera("left")
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        colorCamera.setBoardSocket(dai.CameraBoardSocket.RGB)
        colorCamera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        colorCamera.setVideoSize(1920, 1080)
        # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)

        config = depth.initialConfig.get()
        config.postProcessing.speckleFilter.enable = False
        config.postProcessing.speckleFilter.speckleRange = 50
        config.postProcessing.temporalFilter.enable = True
        config.postProcessing.spatialFilter.enable = True
        config.postProcessing.spatialFilter.holeFillingRadius = 2
        config.postProcessing.spatialFilter.numIterations = 1
        config.postProcessing.thresholdFilter.minRange = 400
        config.postProcessing.thresholdFilter.maxRange = 15000
        config.postProcessing.decimationFilter.decimationFactor = 1
        depth.initialConfig.set(config)
        depth.setLeftRightCheck(lr_check)
        depth.setExtendedDisparity(extended_disparity)
        depth.setSubpixel(subpixel)
        
	# Linking
        monoRight.out.link(depth.right)
        monoLeft.out.link(depth.left)
        monoRight.out.link(xoutRight.input)
        monoLeft.out.link(xoutLeft.input)
        colorCamera.video.link(xoutColor.input)
        depth.depth.link(xoutDepth.input)

    def publish_frames(self):
	# Connect to device and start pipeline
        with dai.Device() as device:
            device.startPipeline(self.pipeline)
	    # Output queue will be used to get the grayscale frames from the output defined above
            qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
            qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
            qColor = device.getOutputQueue(name="video", maxSize=4, blocking=False)
            qDepth = device.getOutputQueue(name="depth", maxSize=8, blocking=False)
            rate = rospy.Rate(30)
            print("Publishing")
            while not rospy.is_shutdown():

                inRight = qRight.get()  
                inLeft = qLeft.get()
                inColor = qColor.get()
                inDepth = qDepth.get()
 
                ros_image = self.bridge.cv2_to_imgmsg(inColor.getCvFrame(), "rgb8")
                self.image_pub_color.publish(ros_image)

                ros_image = self.bridge.cv2_to_imgmsg(inDepth.getCvFrame(), "16UC1")
                self.image_pub_depth.publish(ros_image)
   
                ros_image_left = self.bridge.cv2_to_imgmsg(inLeft.getCvFrame(), "8UC1")
                self.image_pub_left.publish(ros_image_left)

                ros_image_right = self.bridge.cv2_to_imgmsg(inRight.getCvFrame(), "8UC1")
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
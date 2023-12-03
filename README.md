# jetson-yolov8

Simple object detection based on pretrained YOLOv8 model on COCO dataset

In order to run this you must have ROS installed I was working with ros-noetic on wsl

You should put some video as a source for video_publisher and depth_publisher

<b>Python version == 3.8.10</b>

To run script you must do following steps:
- install requirements
- run roscore
- update paths in <b>video_publisher.py</b> and <b>depth_publisher.py</b>
- run <b>video_publisher.py</b> and <b>depth_publisher.py</b>
- run <b>ObjectDetection.py</b>

To test if things are working you should run <b>coords_subscriber.py</b> but for now it does not handle modified video with detected objects, just coordinates
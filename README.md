Skin Detection Algorithm using ROS and OpenCV

Little program which runs a simple OpenCV code and creates a ROS node called skinDetection which transform the input image that comes from the camera in the node called /camera/image_raw.
The skin detection is calculated in the HSV space for simplicity and the resulting image is published in the node called /camera/image_hsv.

For how to run this code into ROS, check out the tutorials in http://wiki.ros.org/ROS/Tutorials

# Creating-a-ros2-package-with-a-node-for-video-conversion-in-ros2
This package contains an image conversion node that subscribes to the USB camera feed and allows switching between grayscale and color image output using a ROS2 service.
   Prerequisites
•	ROS2 ( ROS2 Humble) installed and configured.
•	usb_cam package installed.
•	A working USB camera connected to the system.
Launch the Nodes
To launch the nodes, use the following command:



ros2 launch image_converter image_conversion.launch.py



This will start two nodes:
•	usb_cam_node: This node starts capturing images from the USB camera and publishes them on the topic /usb_cam/image_raw.
•	image_conversion_node: This node subscribes to /usb_cam/image_raw, processes the images based on the current mode (grayscale or color), and publishes the processed image to /image_conversion/output_image.
Change the Image Mode (Color/Grayscale)
The image conversion node supports switching between two modes:
1.	Mode 1 (Grayscale): Converts the input image to grayscale.
2.	Mode 2 (Color): Outputs the image in its original color format.
To change the mode, you can call a ROS2 service.
Example to Set Grayscale Mode
Open a new terminal and run the following command:

ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: true}"


This will set the image conversion node to grayscale mode.
Example to Set Color Mode
To switch the node back to color mode, run:

ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: false}"


View the Output Image
To visualize the processed image, use the rqt_image_view tool:

ros2 run rqt_image_view rqt_image_view


This will display the image published on the /image_conversion/output_image topic, allowing you to see the effect of switching between grayscale and color modes.

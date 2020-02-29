# Purpose
Acquire images from Allied Vision Technologies (AVT) camera using [Vimba SDK](https://www.alliedvision.com/en/products/software.html). Convert to [Robot Operating System](http://www.ros.org) (ROS) image messages.

## ROS Topics
image published with [image_transport](http://wiki.ros.org/image_transport). The root image topics name is ``/avt_camera_img``

The camera can be triggered by sending a message (type ``std_msgs/String``) to ``/trigger`` topic. The camera will acquire an image each time a trigger message is received.

## ROS parameters
``~cam_IP``: type ``str`` default ``169.254.75.133``

``~image_height``: type ``int`` default ``1200``

``~image_width``: type ``int`` default ``1600``

``~exposure_in_us``: type ``int`` default ``10000`` (micro seconds)

``~trigger``: type ``bool`` default ``false``

``balance_white_auto``: type ``bool`` default ``false``

``exposure_auto``: type ``bool`` default ``false``

## Launch files
*image_view.launch*: start a camera in continuous asynchronous grabbing mode.

*image_view_dual_cam.launch*: start two cameras in continuous asynchronous grabbing mode.

*image_view_trigger*: start a camera in triggerd grabbing mode.

## Usage
1. Before installation, make sure the following packages are installed:

[``cv_bridge``](http://wiki.ros.org/cv_bridge)

[``image_transport``](http://wiki.ros.org/image_transport)

2. Clone this package to your working directory

4. Build this package

5. Modify the parameters in the launch files. You need to correctly set your camera IP.

6. Run the launch file, then you should be able to see camera images on the screen.

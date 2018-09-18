# avt_camera
Acquire images from AVT camera using Vimba SDK. Convert to ROS image messages.

## ROS Topics
image publised with [image_transport](http://wiki.ros.org/image_transport). The root image topics name is ``/avt_camera_img``

## ROS parameters
``~cam_IP``

``~image_height``

``~image_width``

``~exposure_in_us``

``~show_frame_info``: type ``bool``

## Launch files
*image_view.launch*

*image_view_dual_cam.launch*

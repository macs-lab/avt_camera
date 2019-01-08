# avt_camera
Acquire images from AVT camera using Vimba SDK. Convert to ROS image messages.

## ROS Topics
image publised with [image_transport](http://wiki.ros.org/image_transport). The root image topics name is ``/avt_camera_img``

## ROS parameters
``~cam_IP``: type ``str`` default ``169.254.75.133``

``~image_height``: type ``int`` default ``1200``

``~image_width``: type ``int`` default ``1600``

``~exposure_in_us``: type ``int`` default ``10000`` (micro seconds)

``~show_frame_info``: type ``bool`` default ``false``

## Launch files
*image_view.launch*

*image_view_dual_cam.launch*

## Usage
1. Befor installation, make sure the following packages are installed:

[``cv_bridge``](http://wiki.ros.org/cv_bridge)

[``image_transport``](http://wiki.ros.org/image_transport)

[``OpenCV 3.4.1``](https://opencv.org/opencv-3-4-1.html) (other version of OpenCV is not tested, might works as well)

2. Clone this package to your working directory

3. In ``CMakeList.txt``, set OpenCV_DIR to your OpenCV build directory:

```
set( OpenCV_DIR /your_OpenCV_dir/build )
find_package(OpenCV 3.4.1 REQUIRED)
```

4. Build this package

5. Modify the parameters in the lanch files. You need to correctly set your camera IP.

6. Run the launch file, then you should be able to see camera images on screen.

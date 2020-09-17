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
### 1. Install the Vimba Driver
Go to the official website https://www.alliedvision.com/en/products/software.html. Download driver for Linux x86/x64 and unzip it. 
Look inside the folder named VimbaGigETL, run `Install.sh` file from command line. After installation, log off once to activate the driver.

Please use `Vimba Viewer` to test if your camera has been properly configered and is discoverable. The Vimba Viewer can be found inside the `Tools` folder. Test if you can open the camera and grab images using `Vimba Viewer`. If not, this ROS wrapper will fail, too.

![alt text](https://github.com/macs-lab/avt_camera/blob/master/doc/vimba_viewer.png)

### 2. Install `avt_camera` package.
Before installation, make sure that [``cv_bridge``](http://wiki.ros.org/cv_bridge) and [``image_transport``](http://wiki.ros.org/image_transport) are installed.
```bash
$ sudo apt-get install ros-melodic-cv-bridge ros-melodic-image-transport
```
Then clone this package to your working directory and build your workspace.

### 3. Run this ROS driver.
The camera can be operated in three modes: fixed rate, freerun, and triggered. You can find example launch files in the launch folder. To start the camera in freerun mode, modify the `cam_IP` parameters in `image_view_freerun.launch` then run the launch file. You should be able to see info messages like below:
```bash
process[avt_camera-2]: started with pid [20715]
process[img_viewer-3]: started with pid [20716]
[ INFO] [1600368569.637844996]: Got camera IP 169.254.49.41
[ INFO] [1600368569.639330100]: Got image_height 1200
[ INFO] [1600368569.639791686]: Got image_width 1600
[ INFO] [1600368569.640252336]: Got exposure_in_us 10000
[ INFO] [1600368569.640806019]: Got frame_rate 5.000000
[ INFO] [1600368569.641288423]: trigger source is FreeRun
[ INFO] [1600368569.641740295]: exposure_auto disabled
[ INFO] [1600368569.642180697]: balance_white_auto disabled
[ INFO] [1600368569.642619134]: gain is 0 dB
[ INFO] [1600368575.595667922]: receiving frame failed.
```
The "receiveing frame faild" info message only indicates a failure of receiveing a single frame. It is normal to see this message from time to time.

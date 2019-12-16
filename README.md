# league_core
Core files for running automated ARCC League

## Dependencies
1. This repo has been developed on and tested for Ubuntu 18.04 with ROS melodic

## dr_interface Setup (not needed unless used seperately from the rest of the system)
1. Start by installing and setting up conda
2. Create a conda env with the command `conda env create -f environmrent.yaml`. That should load all of the proper dependencies for running league core.
3. Activate conda env with with `conda activate league_env` 

## league_ws ROS setup
1. Install ROS melodic
2. init and update git submodules
3. `sudo apt install libunwind8-dev`
4. `sudo apt-get install python-requests python-requests-toolbelt`
5. `rosdep install --from-paths src --ignore-src -r -y`
6. `catkin_make`
   - You may need to download the SDK for the FLIR style cameras manually [follow this](https://answers.ros.org/question/233456/pointgrey_camera_driver-fails-to-compile/)
7. Setup FLIR Camera udev rules by downloading the spinnaker viewer/driver from the link in the setting up the vision system section (software install link) and run the setup making sure to correctly enter the username when prompted (if your camera cannot be detect the reason is likely that the udev rules are not properly setup)
8. In `/etc/default/grub` add the line `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"` then run `sudo update-grub`
9. Calibrate Camera with tutorial [here]("http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration")

## Getting Started
Once you have followed the setup instructions you can begin running ACC League software!

## Setting up the vision system
ARCC league uses quality machine vision system to provide reliable DR tracking. The follow links are to the setup instructions for the [Flir BFS-U3-16S2C-CS](https://www.flir.com/products/blackfly-s-usb3?model=BFS-U3-16S2C-CS) Camera.

[Lens YV2.7X2.2SA-2](https://www.bhphotovideo.com/c/product/736834-REG/Fujinon_YV2_7X2_2SA_2_3_MP_Varifocal_Lens.html)

[Setup Instructions](https://flir.app.boxcn.net/s/4nmu4yffg9h7qov46w5ijcude99nks0u/file/418603801042)

[Software Install (Spinnaker)](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/68522911814)

[Understanding Linux and USBFS](https://www.flir.com/supeport-center/iis/machine-vision/application-note/understanding-usbfs-on-linux/)

The specs for the camera when focus = infinity and the zoom in minimized is the following
HFOV: 120
VFOV: 90
Frame size: 1440(h)x1080(w)
FPS: ~30
Topic: /camera_array/cam0/image_raw

For camera calibration the following command can be used: `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.038 image:=/camera_array/cam0/image_raw --no-service-check`

('D = ', [-0.21338195826140682, 0.03258095763463949, -0.00018955724409316463, -0.00040928582228521367, 0.0])
('K = ', [648.0113147742057, 0.0, 703.9625700394782, 0.0, 648.1713061967582, 533.5561962879885, 0.0, 0.0, 1.0])
('R = ', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
('P = ', [445.3047180175781, 0.0, 707.8064656754395, 0.0, 0.0, 526.1232299804688, 529.175376518222, 0.0, 0.0, 0.0, 1.0, 0.0])


## Launch instructions
Nodes and functions can be launch individually or together. Launch files package contains the large single files that will launch every component of the system. Parameters to the launch file include which tracking system to use and the config files to be loaded for each process. The rqt_graph for the single car system is shown below.

Check all configuration files are up correct
1. dr_tracker_config.yaml
2. track_util_config.yaml
3. ekf_config.yaml
4. [base_local_planner_params.yaml](http://wiki.ros.org/teb_local_planner#Parameters)
5. costmap_common_params.yaml

Make sure you have the following:
1. Calibrated wide FOV machine vision camera
2. Map corresponding to camera placement and track layout
3. Updated parameters for ekf based on camera, lighting, and map

## Generating Map and updating state covariances for EKF


## Updating tracker and track_utils config files

## Tracking System Configuration
Two types of tracking systems exist. One is a traditional motion system that ses computer vision techniques to detect and track a triangle. The other technique uses object detect neural networks and computer vision to detect, orient, and track the obect.

Setting up triangle tracking
1. Design an isosceles triangle with a 9/4 height to base ratio. The color should be one that is not encountered in large amounts on the track such as red or blue. The mask can then be tune in the dr_tracker/config/dr_tracker_config.yaml along with various other data points needed for accurate calibration of the tracking system.
2. Existing triangle files can be found [here](https://drive.google.com/drive/u/0/folders/1nh8eqmYK21Rf7553yW1CQx-jX22Z4VH3)
3. Best performance has been found with 9/4 ratio dark red/blue triangles printed on thick matte paper.
4. Example dimensions for a triangle that fits the DeepRacer frame would be h:275mm and b: 122mm with RGB values (158, 11, 15)

Setting up Deep Neural tracking
1. print out on thick matte paper colored buttons (circles) with clear delinitations between in and the rest of the cars shell and other colored dots. For example colors could be purple, blue, green, red, yellow. For the AWS Deepracer the dots should have a diameter of 50mm.
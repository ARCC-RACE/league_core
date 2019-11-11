# league_core
Core files for running automated ARCC League

## Dependencies
1. This repo has been developed on and tested for Ubuntu 18.04 with python 3.7

## dr_interface Setup 
1. Start by installing and setting up conda
2. Create a conda env with the command `conda env create -f environmrent.yaml`. That should load all of the proper dependencies for running league core.
3. Activate conda env with with `conda activate league_env` 

## league_ws ROS setup
1. Install ROS melodic
2. init and update git submodules
3. `rosdep install --from-paths src --ignore-src -r -y`
4. Calibrate Camera with tutorial [here]("http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration")

## Getting Started
Once you have followed the setup instructions you can begin running ACC League software!

## Setting up the vision system
ARCC league uses quality machine vision system to provide reliable DR tracking. The follow links are to the setup instructions for the [Flir BFS-U3-16S2C-CS](https://www.flir.com/products/blackfly-s-usb3?model=BFS-U3-16S2C-CS) Camera.
[Lens YV2.7X2.2SA-2](https://www.bhphotovideo.com/c/product/736834-REG/Fujinon_YV2_7X2_2SA_2_3_MP_Varifocal_Lens.html)
[Setup Instructions](https://flir.app.boxcn.net/s/4nmu4yffg9h7qov46w5ijcude99nks0u/file/418603801042)
[Software Install (Spinnaker)](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/68522911814)
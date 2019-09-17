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
2. `rosdep install --from-paths src --ignore-src -r -y`
3. Calibrate Camera with tutorial [here]("http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration")

## Getting Started
Once you have followed the setup instructions you can begin running ACC League software!
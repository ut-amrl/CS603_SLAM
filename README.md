# CS603_Particle_Filter
Particle filter for mobile robot localization using a laser rangefinder.

## Dependencies
Install [ROS](http://wiki.ros.org/ROS/Installation)

And install additional dependencies
   ```
   sudo apt-get install cmake build-essential clang libgoogle-glog-dev  libgflags-dev libgtest-dev
   ```

## Clone This Repository
   ```
   git clone --recurse-submodules git@github.com:umass-amrl/CS603_SLAM.git
   ```
   **IMPORTANT:** The `--recurse-submodules` is required to clone the shared
   library submodule.

## Build Instructions
1. Clone this repo, and enter the directory. 
   All subsequent commands must be run from within the directory.
1. Add the project to the ROS packages path:
   ```bash
   export ROS_PACKAGE_PATH=/PATH/TO/YOUR/REPO:$ROS_PACKAGE_PATH
   ```
   Pro-tip: add this line to your `.bashrc` file.
1. Run `make`.  
   DO NOT RUN `cmake` directly.  
   DO NOT RUN `catkin_build` or anything catkin-related.  
   
   ```
   ## Run 
1. Before starting programs that use ros, you will need to start a roscore instance in a terminal that you keep open:
     ```
   roscore
   ```
1. To run the particle filter with a bag file named `myfile.bag`:
   ```
   ./bin/slam-backend --input myfile.bag
   ```
1. To visualize the particle filter in RViz, run the following while also running your filter:
   ```
   rosrun rviz rviz -d visualization.rviz
   ```

## Example Bag Files
An example bag file for you to test with can be downloaded [here](https://drive.google.com/open?id=1I_cD2bPP5PsKJh30PgvIrtw9iT3Rpdu4). Please make sure you do not include the bag file with your submission.


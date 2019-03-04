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
   
## Run 
1. Before starting programs that use ros, you will need to start a roscore instance in a terminal that you keep open:
     ```
   roscore
   ```


# Algorithm 1: Shifted Line Following

# Additional Packages
This package requires the `SteeringReport` and `UlcReport` messages from the drive-by-wire (DBW) system.
Repo: https://bitbucket.org/DataspeedInc/dbw_polaris_ros/

## Install DBW Messages
### 1. SteeringReport
Follow all the steps [here]( https://bitbucket.org/DataspeedInc/dbw_polarisyy_ros/src/f229dcb6a9366524e1aacc2e80e4df2fd7995143/ROS_SETUP.md).

### 2. UlcReport
You show now have a `dbw_ws` in your home directory. This workspace contains the `SteeringReport` message, to install the `UlcReport` message, run:
```
cd ~/dbw_ws/src
git clone https://bitbucket.org/DataspeedInc/dataspeed_ulc_ros.git
```

### 3. Build
Source and catkin_make the packages in the `~/dbw_ws` directory:
```
cd ~/dbw_ws
source devel/setup.bash
catkin_make
```

### 4. Test
The drive-by-wire system `steering_report` and `ulc_report` messages should be built and ready to use. To test run a rosbag with the `steering_report` and/or `ulc_report` topics and run:
```
rostopic echo /vehicle/steering_report
```

This will only work if the `~/dbw_ws/devel/setup.sh` is sourced or if you sourced `dbw_ws` before running `catkin_make` on the workspace where this package is located.


### NOTE
Make sure you source `~/dbw_ws/devel/setup.sh` **before** running `catkin_make` on the current workspace.

If this still gives you problem, delete the `~/<your_ws>/build` directory and rebuild the workspace:
```
rm -rf ~/<your_ws>/build
source ~/dbw_ws/devel/setup.sh
catkin_make
source ~/<your_ws>/devel/setup.sh
```

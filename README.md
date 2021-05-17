# Collision Free Configuration
Before start, make sure that you have installed:

```
pip install Shapely==1.7.1
```
The following pybullet_planning package is a little different from python pybullet_planning package.
```
git clone https://github.com/liwei2998/pybullet_planning_new.git
```

Launch your robot in virtual environment, the following command is just an example.

```
$roslaunch dual_arm_moveit demo.launch
```

Generate collision free configurations and choose the max distance configuration.

```
$python config_generalization.py
```

# Pybullet Setting
This repo is tested with python 2.7 and pybullet 2.5.6. Install pybullet with the command

```
pip install pybullet
```

To load the pybullet simulated environment for this lab as shown below, simply run

```
python demo.py
```

Because Pybullet only accept .urdf files, run combineURDF.py to transform your .xacro files.

# Simple Demo

1. create a new workspace and extract the two .zip files to the workspace, $catkin_make
2. Launch your robot in virtual environment, the following command is just an example.

```
$roslaunch dual_arm_moveit demo.launch
```
3. Simply run

```
python autonomous fix demo.py
```

# Demo Explanation
This demo shows how the hong arm finds the collision-free configuration for fixing. In the main function, first the hong kong arms go the the home position;
Then, in flexflip function, kong arm moves to the grasping position; the fix function shows the logic of finding a collision-free configuration, please see the
code comments for explanation. 

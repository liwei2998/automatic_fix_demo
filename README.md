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

# Roomba on ROS

Simulating iRobot's Roomba behavior using ROS.

Move simulation files to the package folder.
```
mv roomba/stage_ros/* $(rospack find stage_ros)/world/
```

Launch it!
```
roslaunch roomba roomba_house1.launch
```


# GPD_FFC
Grasp Pose Detection with Frictional Force Compute (GPD_FFC)


## Overview

This project extends the application of GPD.
We compute the frictional force of grasp candidates and use the MoveIt! program to move robot.


## Usage

1. Use original GPD program to generate grasp candidates.

2. Compute the frictional force of each grasp candidates, then output the GraspConfig message of highest FFC score candidate.
    ```
    rosrun gpd frictional_force_compute
    ```

3. Received the GraspConfig message from the previous program, then move robot to goal position. 
    ```
    rosrun gpd FFC_get_grasps.py
    ```


## Reference

[1] Andreas ten Pas, Marcus Gualtieri, Kate Saenko, and Robert Platt. [**Grasp Pose Detection in Point 
Clouds**](http://arxiv.org/abs/1706.09911). The International Journal of Robotics Research, Vol 36, Issue 13-14, 
pp. 1455 - 1473. October 2017.

[2] Marcus Gualtieri, Andreas ten Pas, Kate Saenko, and Robert Platt. [**High precision grasp pose detection in dense 
clutter**](http://arxiv.org/abs/1603.01564). IROS 2016. 598-605.

[3] Andreas ten Pas. [**GPD**](https://github.com/atenpas/gpd)

# ExploEnv

This repository contains the basic environment code for autonomous exploration.

## Topics You May Use

* `/visible_map`: the current mapping result.
* `/ground_truth`: the actual underlying world without sensor noise or occlusion.
* `/av1/odom`: the odometry.
* `/av1/cmd_vel`: the twist of robot.

It is suggested to put all robot-related topic into `av1` namespace.

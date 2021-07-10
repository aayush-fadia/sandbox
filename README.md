# Sandbox
A playground for testing out algorithms on a robot.

A robot is dropped into a realistic world that mimics an apartment.
The robot has a depth/rgb camera, a LIDAR and a differential drive base.
The simulation exposes all of this as ros2 topics.

All of this is nicely encapsulated into one docker container, which launches PyCharm for full code assistance.

Refer to all the packages in the ros workspace src, to see individual docs.

# Algorithms Implemented
* Primitive visual odometry (ongoing) [Video](https://drive.google.com/file/d/1AzDaz9ERiqZGXREHXc0hbzQmHyg_6-aH/view?usp=sharing)
* Robot pose estimation w.r.t ArUCo markers (ongoing)
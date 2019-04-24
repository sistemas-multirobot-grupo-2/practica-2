# 2nd practical exercise
This repository containt the files related with the 2nd practical exercise of the Multi-Robot Systems, a subject of the 4th year from the Robotics Engineering Degree of the University of Alicante.

The final purpose is to implement multi-robot behaviours using Turtlebots robots. Our first goal is to archive a computer vision based leader-follower behavior. To archieve this, so far we have designed a **3D simulation** using ROS and Gazebo in which two Turtlebots are launched in an empty world, with their models modified so that they are equiped with a UST-10 LIDAR, an Orbbec Astra RGB-D camera and a green plate on their back. This way, they will be mostly identical to the devices we use in our laboratory:

![Basic simulation](https://github.com/sistemas-multirobot-grupo-2/practica-2/blob/master/multimedia/plain_simulation.png "Basic simulation")

This can be launched by adding our "turtlebot" folder in the "src" directory of a catkin workspace in a machine with a functional ROS installation by executing:
```bash
roslaunch a_multirob create_multi_robot.launch
```

In order to launch the **perception node** (*perception*), it is necessary to use the *a_multirobot* package. The order to run it is:
```bash
rosrun a_multirob perception
```
This node takes the images published in the "robot2/camera/rgb/image_raw" topic and gets the bounding box and centroid of the biggest green area on the frame. Now, it just displays it in a window, but we are working to publish this relevant data in topics ("/area", "/centroid" and "/bounding_box").

![Perception node](https://github.com/sistemas-multirobot-grupo-2/practica-2/blob/master/multimedia/perception.png")


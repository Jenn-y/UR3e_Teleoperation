## UR3e Robotic Arm Teleoperation with Meta Quest 2 

Group project for Robotics & XR course 2023 (IMLEX & COSI Erasmus Mundus Programs)

RXR_project folder contains Unity app that records the coordinates of VR controller, calculates the joint angles via inverse kinematics and sends the angle values via http request to a Python API.
robotArmManager folder contains Python API that receives angle values, records them in a file that will be read by RTDE and sent to the physical robot to move.

Installation of *git lfs* recommended for using the Unity app.

Group members:
- Kuldeep Dileep
- Marta Margherita Pozzi
- Louis Vanhoonacker
- Dženita Đulović

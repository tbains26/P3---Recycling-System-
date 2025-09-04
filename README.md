# P3---Recycling-System-
Developed a dual-robot recycling system algorithm as part of our third engineering design project, implemented in Python on a Raspberry Pi 4 and simulated using Quanser Interactive Labs.
# Overview of the Project
Our programming workflow began with the sorting station dispensing a random container. The robotic arm would pick up the container, measure its weight, and group up to three identical containers onto the hopper of the transfer robot. Based on the combined weight, the transfer robot determined which sorting bin the load should be sent to. Paper, plastic, and metal containers were directed to separate bins, while paper and plastic containers exceeding the weight limit were considered too dirty for recycling and instead sent to the garbage bin.

The transfer robot navigated along a predefined path using a line-following algorithm. A color sensor and an ultrasonic sensor worked together to identify the correct bin where it should stop and unload. After dropping off the containers, the robot continued along the path until it returned to its home position. This cycle repeated continuously until the program was stopped.

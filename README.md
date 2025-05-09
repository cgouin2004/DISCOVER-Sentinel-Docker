# DISCOVER-Sentinel-Docker
This is the DISCOVER Sentinel Drone Docker with all experiments, scripts, etc.

Experiments are located in Docker/source/connorscode/src
Main flight module is SGFlightKit Core which includes a variety of flight function with different use cases.
These functions are documented on the github but not yet on the drone itself.

Many experiments may be unfinished or outdated given the current parameter set. This can cause a crash, 
so thouroughly read through the experiments and all embedded function before flying, and do simple tests
before jumping into a complex script.

To Run Experiment:
1. Connect to Drone’s wifi signal (password: 1234567890)
2. Open terminal
3. ssh root@DRONEIP
4. Enter password: oelinux123
5. Cd /data/bryce_docker_testing/Docker
6. ./run.sh
7. Cd /ros_ws/src/scripts
8. ./run_mavros.sh
9. New terminal
10. ssh root@DRONEIP
11. Enter password: oelinux123
12. Cd /data/bryce_docker_testing/Docker
13. ./open_second_terminal.sh
14. Verify that you are in directory ros_ws
15. Catkin_make --only-pkg-with-deps connorscode (or whatever package is being used)
16. Souce devel/setup.bash
17. Cd src/connorscode/src
18. Rosrun connorscode EXPERIMENT_FILE_NAME.py
19. To end code use ctrl-z

You may get a ROS_IP error related to mavros or voxl-mpa-to-ros in this case you have to export ROS_IP=DRONEIP (this is only a temporary fix)

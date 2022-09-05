# Do catkin build and sourcing it for making a workspace for the MiRo demo to work
cd ~/mdk/catkin_ws/
catkin build
source devel/setup.bash
# Launching the Demo
cd ~/mdk/catkin_ws/src/speech_recognition_porcupine/launch
pwd
roslaunch speech_recognition_porcupine miro_porcupine_demo.launch


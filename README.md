# miro2_examples
A collection of Python scripts for MiRo intended for testing ROS packaging within the MDK 

# List:

## Controlling miro with the keyboard
Simple example of using the keyboard to control MiRo

## Human detection
Example of using miro to detect pedestrians

## Obstacle avoidance + depth
Using QLearning for obstacle avoidance + depth perceptioning (Still being curated)

## Rectifying the camera
Rectify the image removing the distortion from miro eyes (Still being curated)

## Kinematic joints
Usage:
1. In your .bashrc, after 
`source ~/mdk/setup.bash`
add
`source ~/mdk/catkin_ws/devel/setup.bash`
2. Clone this repo into `~/mdk/catkin_ws/src`
3. `cd ~/mdk/catkin-ws $$ catkin_make`
4. Restart the terminal
5. Make the files inside `nodes` executable, e.g. `chmod +x joints`
6. Now, you'll be able to do something like `rosrun miro2_examples joints`



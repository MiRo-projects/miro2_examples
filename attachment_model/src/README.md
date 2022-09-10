# Attachment Model

## Actions
The following part of the code contains messages to be published for the MiRo to take actions. It also contains a sequence of actions that may be executed by the MiRo depending on the behaviour it is supposed to exhibit.
## Audio Detection
The following part of the code contains speech detection of another MiRo. The ```actions.py``` in this is set as an independent node that will be run locally in each MiRo so as to decide whether audio has been recognised or not. Since this function requires a specific rospy rate and rest that may affect other functionalities, this part of the function has been separated from the others.
## Controller
The following part of the code contains the controller to calculate on the attachment model itself. It contains separate emotional and physical controller to calculate the distances depending on whether the MiRos are supposed to run together or individually. This will then be processed by the ```robot_controller.py``` to decide what course of actions should be taken. The controller can be divided for each MiRo or to take command over both MiRo.
## Run
The following part of the code contains ros node codes to be run by the MiRos for each part of the project.
## Subscriber
The following part of the code contains a list of subscribers required by the MiRos to collect the required information. It may also be subscriber to redirected odom data that is specific to the parent or child. This is required when both MiRos work together in the same controller.
Audio Detection On MiRo
=======================

Introduction
------------

<p>&nbsp;&nbsp;&nbsp;&nbsp;The following project will be making use of a robot from Consequential Robotics called the MiRo to produce capabilities of audio detection on it. There are two parts to the project which will be to focus on audio detection of words for the Miro and to detect the "coo" of the MiRo, which is a form of specialised animal-like sound made by the MiRo.</p>
<p>&nbsp;&nbsp;&nbsp;&nbsp;The first part of the project will be focusing on the audio detection of wake words. This will be making use of a light-weight module called porcupine. The second part of the project will be focusing on designing a neural network model so as to process the audio. The training data of this will be focused on the MiRo's "coo", which will either be a happy "coo" or a sad "coo".</p>

Usage
-----
### Porcupine on MiRo:
Audio detection for miro can be found in `sound_recognition_porcupine.py` where the audio detected from the left ear speaker of the MiRo will be used for processing where the word has been detected or not. This will then be published to `$MIRO_NAME$/audio/detected_words`. The following is the ROS message:
```
string detected_words
```
The following words are valid wake words to be used by the MiRo:
- "Hey MiRo"
- "No MiRo"
- "Good Boy"

Demo
----
<p> The demo will consist of the MiRo doing a set of actions as a response to the wake word it has detected</p>

 - Clone this repository into `~/mdk/catkin_ws/src`
 - Go into the directory and run
 ```
 ./porcupine_miro_demo.sh
 ```
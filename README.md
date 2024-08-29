# Vision-Based Hand Gesture Controlled UGV

In progress ....

### Recipe
ESP32, cheap chassis off amazon, micro ros, ROS2, Mediapipe, opencv, maybe some ui.

### Plan
Micro ROS runs on ESP32 (ugv) , subscribes to a custom topic for commands on when to move. 
Use Media pipe, get landmarks and specify labels, train a classifer, recognize hand gestures. 
Create a ROS2 visual_teleop node, construct custom msg based on hand gesture, publish over custom topic.

### Branches
uros_dev : For miro ros stuff
vis_dev : for gesture recognition stuff.

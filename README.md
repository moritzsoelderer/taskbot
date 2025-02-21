**TaskBot**

Human Robot Interaction
LMU WS24/25

**Group:**
- Lukas Petz
- Moritz Sölderer
- Violetta Meier

**Idea: TaskBot, the Kitchen Assistance Robot**

The idea behind TaskBot is to create an interactive kitchen assistance robot. This robot is designed to help in the kitchen by performing simple tasks, like grabbing utensils or tidying up. It is a helpful solution for making kitchen work more efficient and organized. 

**Use case:**

Tidying up objects after use. Recognizing whether a person is finished with a task (camera and user feedback). E.g. knife being used to cut vegetable --> knife no longer in use --> knife put in sink

**Milestones:**

1. Recognize items (Camera calibration, AprilTags)
2. Grasp objects (Movement of joints and grasping)
3. Recognize task state (logic, LLM)
4. Interactively help user

**Implementation:**
- 3D print items --> add Apriltags for position & recognition
- Robot grasps items
- LLM: queried to get task state (in progress/done)
- Query user for feedback: "Do you still need this?" --> "Yes"/"No"
- Robot act interactively depending on state

**Hardware needed:**
- Robot
- 3D printed kitchen item: knife
- Camera
- Speaker
- Microphone
- Print: AprilTags, Calibration checker board

**Components:**
- Voice Control
- Camera coords
- Robot Control (joints, gripper)
- LLM

**Software Components:**
- Sensors:
    - AprilTags
    - Speech Recognition
    - Text-to-speech
- Objects
    - define positions (marked with AprilTag)
- State machine
    - define logical scenario

**HOW TO: Add new script**
- Create script (<em>your_script.py</em>) inside of a catkin package under <em>your_package</em>/scripts
- Run `catkin_make` (optional?)
- Make script executable: `sudo chmod +x your_script.py`
- If necessary allow permission for usb-port: `sudo chmod 666 /dev/ttyACM0`
- Run script: `rosrun your_package your_script.py`

**HOW TO: Install required packages**
run `pip install -r requirements.txt`

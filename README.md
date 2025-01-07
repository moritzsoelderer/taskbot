**TaskBot**

Human Robot Interaction
LMU WS24/25

**Group:**
- Lukas Petz
- Moritz Sölderer
- Violetta Meier


**Idea:**
- Raster with Items
- Prompt
- Camera
- LLM

**HOW TO: Add new script**
- Create script (<em>your_script.py</em>) inside of a catkin package under <em>your_package</em>/scripts
- Run `catkin_make` (optional?)
- Make script executable: `sudo chmod +x your_script.py`
- If necessary allow permission for usb-port: `sudo chmod 666 /dev/ttyACM0`
- Run script: `rosrun your_package your_script.py`
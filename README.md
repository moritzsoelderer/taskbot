**TaskBot**

Human Robot Interaction
LMU WS24/25

**Group:**
- Lukas Petz
- Moritz Sölderer
- Violetta Meier


**Idea: TaskBot, the Kitchen Assistance Robot**
The idea behind TaskBot is to create a kitchen assistance robot. This robot is designed to help in the kitchen by performing simple tasks, like grabbing utensils or tidying up. It is a helpful solution for making kitchen work more efficient and organized. 

**Use case 1 :**
Gives person matching items to the one just used. E.g. cutting board --> knife

**Use case 2:**
Tidying up objects after use. Recognizing whether a person is finished with a task (images or user feedback). E.g. used knife --> sink

**Milestones**
1. Recognize currently used items
2. Grasp objects
3. Recognize task state
4. Interactively help user 

**Implementation:**
- 3D print items --> add QR-Codes for position & recognition
- Grid for items
- Robot grasp items
- LLM: queried to get task state (in progress/ done)
- Query user for feedback: "Are you finished?" --> "Yes"/"No"
- Robot act depending on state

**Hardware needed**
- Robot
- 3D printed items (kitchen items)
- Camera
- Speaker
- Microphone
**TaskBot**

Human Robot Interaction
LMU WS24/25

**Group:**
- Lukas Petz
- Moritz Sölderer
- Violetta Meier


**Idea:**
Kitchen Assistance robot
Helps in the kitchen
Grabs Items

**Use case 1 :**
gives person matching items to the one just used
E.G. Cutting board --> Knife

**Use case 2:**
Tidying up objects after use
Recognizing whether a person is finished with a task (images or user feedback)
E.g. used knife --> sink

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
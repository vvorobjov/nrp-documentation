Step 1: show something to the robot
===================================

Context
^^^^^^^

This is the virtual lab of the Neurorobotics Platform. If you press the "play" button (top left), you will see the robot move a bit and converge to his resting position... but nothing more. That's all right, we will fill this experiment with files to see what is happening in our robot's brain.


The State Machines
^^^^^^^^^^^^^^^^^^

This tutorial highlights the visual system of the robot with the brain visualizer. The first thing we want to do is to display some visual stimulus on the screen. To have something a bit living going on (something moving), we use a state machine. The state machines define a set of events that recursively happen throughout the simulation. First, open the state machine editor by clicking on the green circle-shaped button (4th from the top).

We will upload a pre-written state machine. Click on the "upload" button of the state machine editor. Then, browse to the materials folder and upload the file number 1 (1_display_stim.exd). After the file is loaded, the simulation is paused. If you click on the "play" button again, some squares appear and disappear.

The text of the state machine is now visible. After some imports, the position and shape of the stimulus are described and then two classes are defined: one to make the stimulus appear and another one to make it diseappear. You can play with the file if you want (don't forget to validate the changes that you make to the text with the "apply" button), or you can go on with step 2.
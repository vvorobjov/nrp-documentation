Introduction
============

The Virtual Coach is a Python API that allows you to run and interact with experiments by scripting them instead of having to use the Web Cockpit. It is ideal for running learning experiments, where usually one experiment has to be run multiple times, each time with a different parameterization, and where intermediate results have to be saved to compare the effectiveness of different parameters at the end. However, while an experiment is running from the Virtual Coach, you may still open the frontend and visualize what is happening in you experiments.

Users now can launch experiments from the Virtual Coach, interact with the simulation by adding, deleting or editing Transfer Functions and State Machines as well as modify the Brain Model and the Neural Populations on the fly. Also, CSV data that is being saved during a simulation can be accessed from the Virtual Coach and you can then plot the data using your own plotting functions for example. Additionally, you can reuse the same reset functionality found in the Web Cockpit from the Virtual Coach, meaning that after certain events or after running a simulation for a certain amount of time you can either reset the robot pose, the brain model, the environment or reset the whole simulation.

In the next page you will find a description of the API, and if you have the local Virtual Coach repository you can check out the VirtualCoach/examples directory for some examples. Note that examples in the repository may be saved in a jupyter notebook as it makes it easier to run everything step-by-step and visualize results in place.


.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>
   
=======
Welcome
=======

Thank you for your interest in the Neurorobotics Platform (NRP). After reading this documentation, you will have become familiar with how the NRP operates and you will be ready to start working with it.

General Introduction to the NRP
===============================

The Neurorobotics Platform's purpose is to offer neuroscientists a tool to perform in-silico cognitive or lower-level neural experiments on virtual Guinea pigs, be them biologically inspired or not. It will, on the other side, provide roboticists with the possibility to experiment on their robots with brain models instead of classical controllers.


New features of NRP v4.0: a historical perspective
==================================================

The software development of the NRP started in 2013, at a time when concepts such as process isolation and containerization had not yet become widespread. In these early years, a fairly monolithic architecture was therefore chosen, consisting in a synchronization component (the so called closed-loop engine, or CLE) orchestrating communications between two simulators tightly integrated into the NRP code: Gazebo and NEST. Another foundational choice was the use of ROS as a communication layer between these various components. And so the legacy NRP was developed up to version 3.2.

It became clear, however, that the static nature of this legacy architecture compromised both long-term performance and sustainability of the NRP insofar as continuously evolving domain, user and technology requirements were bound to erode the usefulness of the Platform. It was thus decided to proceed with a complete redesign, with a view to make the NRP intrinsically capable of evolution. The result of this work is version 4.0 of the NRP, with version 3.2 referred to as "legacy NRP". 

The fundamental structural principle of NRP 4.0 is modularity. Simulations in the NRP now implement a hub-and-spoke architecture, whereby a central “hub” component (referred to as NRP-core) orchestrates the execution of all others simulation-related components. These “spoke” modules can be heterogeneous in nature and function, and their number is not prescribed. This enables the NRP to use a wide variety of simulators (e.g., Gazebo, NEST, Unity, MuJoCo, PyBullet, EDLUT, OpenSim) and create complex control architecture by composing various components. 

NRP-core also keeps the essence of the transfer function (TF) framework offered by the legacy NRP. Users can thus decouple the computational features of individual modules from the definition of the connections that exist between them. Finally, a new API allows easy integration of NRP-based experiments into standard learning frameworks (e.g., Stable Baselines), e.g., in place of OpenAI Gym environments.


.. _contact:

.. include:: contact-and-support.rst

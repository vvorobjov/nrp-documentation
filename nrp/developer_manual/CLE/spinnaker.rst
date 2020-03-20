=============================================
Using Spinnaker in the Neurorobotics Platform
=============================================

Spinnaker is a neuromorphic hardware capable to run neuroscientific experiments on dedicated hardware
in real time. The NRP has specific support for Spinnaker communicating over Ethernet.

Installing Spinnaker
--------------------

The SpiNNaker framework is already installed in the NRP. If you are installing from docker, just consider the `post-installation`_ step
to configure a board that would *not* be the 4-chip SPiNNaker 3 board or if you have network specifics. Else, the defaults settings
will just work fine.

From the NRP frontend page, go to the templates and clone any of the two SpiNNaker experiments (use the filter with keyword "spinnaker").
Then you can browse the experiment files using the "Files" button to figure out how to set up a SpiNNaker experiment.

.. _post-installation: https://neurorobotics.net

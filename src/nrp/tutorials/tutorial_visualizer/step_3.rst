Step 3: fixing the brain file
=============================

.. todo:: Add author/responsible


Context
^^^^^^^

We have a big brain to process the stimulus, but we still need to record its spikes. Close the brain editor, but let the brain visualizer open. Then open the transfer function editor, to define the spike recorders for all our populations. It's the green button that looks like two plugs.


Defining new transfer functions to record some spikes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

În the transfer function editor, we already have one transfer functions that you can let as it is. The first thing we want to do is to upload a transfer function that feeds the network with the image that is recorded by the camera of the robot. Click on the "upload" button and upload the file: 3_grab_image.py. Every time the message appears on the screen, click on "add" and never on "replace".

This transfer function feeds the first layers of the network (LGN layers) with the camera output. Click on apply to take the change into account. Still no spike? That's ok. What we need to do next is to record the spikes of the populations. So click on the "upload" button again to upload the transfer functions numbered from 4a to 4e. Each transfer function records the spikes occuring in a specific layer of the network.

After you uploaded all the transfer functions, you need to click on "apply all" to take the changes into account. This takes a while to load, because the NRP needs to create a spike recorder for every neuron in the network. But in the end, you start to see some spikes happening (don't forget to move the spike contrast cursor).

However, even if we play with the different display modalities of the brain visualizer, the spike pattern never really looks like what we would expect. We would like to see the actual stimulus encoded in the brain, just as in the figure of step 2! To do so, we need to customize the brain visualizer a bit. This happens at the next step.

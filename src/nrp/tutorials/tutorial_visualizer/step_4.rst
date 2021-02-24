Step 4: debugging performance issues
====================================

Context
^^^^^^^

We are now recording all the spikes of the network, but the shape they highlight does not give a lot of insight about what is happening in the network. We need to customize the view of the brain visualizer.


Custom brain visualizer
^^^^^^^^^^^^^^^^^^^^^^^

We are going to upload a customized configuration .json file for the brain visualizer, that contains the position of each neuron. The file is pre-written, but you could as well make your own script that reads the brain file and builds a different .json file.

The first thing to do is to leave the virtual experiment, because we are updating the setup file of the experiment (to make the reference to the new brain visualizer configuration file). Don't worry: the experiment saves everything you did up to now.

After leaving the experiment, click on the "Files" button of our experiment. You will be redirected to the list of all files that have been created to make the experiment work. The first thing we are going to do is to upload the .json configuration file. Click on the "upload file" button (top right of the list), and choose the file: 5_neuron_positions.json.

The second thing we need to do is to make the reference to the custom configuration file, inside the setup file of the experiment. First, download this setup file, by hitting the "download" button of the line that goes after the file "experiment_configuration.exc". Then, go in your download folder and update the file by adding the following line after the other line that begins with "<configuration ...".

.. code-block::html

    <configuration type="brainvisualizer" src="5_neuron_positions.json"/>

Then upload this modified file just as you did for the .json configuration file. When the dialog box appears, choose "yes", as it will update the already existing .exc file. Then, you can go back to the experiment list by clicking on the tab "My experiments" and launch the experiment again. This might take a while this time, because all the neurons and spike recorders have to be created.

After launching the experiment, hit the "play" button and open the brain visualizer again. If everything went right, the "custom" option should be available in the brain visualizer and the neurons and their spikes should be displayed with the same structure that appears in the figure of step 2.

You can see the input brightness and darkness being first encoded in the LGN layers, then the vertical and horizontal features being detected in the different V1 and V2 layers and finally surface signals spreading in the different V4 layers. The spikes are updated as the stimulus changes on the screen.

That's it! You now master the brain visualizer and its customized version as well.

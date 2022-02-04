.. index:: pair: page; Running the example experiments
.. _doxid-running_example_exp:

Running the example experiments
===============================

In this section, we show how to run some of the example experiments. Different experiments utilize different features of the NRP, the use of which will be highlighted.

Experiments are run using the provided application :ref:`NRPCoreSim <doxid-nrp_simulation>`. It takes as arguments the :ref:`simulation configuration file <doxid-simulation_configuration>` used to configure the experiment and, optionally, a list of :ref:`engine plugins <doxid-plugin_system>` to be loaded at run time. Please notice that by default only :ref:`Python JSON Engine <doxid-python_json_engine>` is loaded. Any other plugin required to execute an experiment must be explicitly added with the "-p" parameter. See examples below.



.. _doxid-running_example_exp_1getting_started_experiment_husky:

Husky braitenberg
~~~~~~~~~~~~~~~~~

This experiment replicates the omnipresent *Husky Braitenberg* experiment often used in previous versions of the NRP to exemplify its functionality. More details about the experiment can be found `here <https://neurorobotics.net/Documentation/nrp/user_manual/user_interface/introduction.html#template-experiments>`__. It displays a virtual robot connected to a brain reacting to color stimuli.

The experiment engages two simulators, `Gazebo <http://gazebosim.org/>`__ as robot simulator, and `NEST <https://www.nest-simulator.org/>`__ as neural network simulator.

Both Gazebo and NEST are wrapped into so-called :ref:`engines <doxid-engines>`, which allow the NRP to manage simulation execution and synchronization. Data exchange between the simulators is done with the help of :ref:`transceiver functions <doxid-transceiver_function>` and :ref:`datapacks <doxid-datapacks>`. In short, the transceiver functions allow to translate output of one simulator into input of another one, while the datapacks serve as generic containers for the data.

The Gazebo engine starts Gazebo server (gzserver), so you should be able to peek into the simulation by connecting with Gazebo client (``gzclient``) from a separate terminal.

To run the example, if you haven't done it yet, you have to install first some additional Gazebo models. Instructions on how to do it can be found in this page: :ref:`Additional Models for Braitenberg Husky experiments <doxid-tutorial_helpful_info_1tutorial_helpful_info_husky>`. Once this is done, switch to the proper directory and run the NRP simulation executable:

.. ref-code-block:: cpp

	cd examples/husky_braitenberg
	NRPCoreSim -c simulation_config.json -p "NRPGazeboGrpcEngine.so,NRPNestJSONEngine.so"

*Please note that the NRPCoreSim must be executed from the example directory!*

Here is a short description of all files that are located in the example directory:

* braitenberg.py - contains the setup of the neuronal network running the experiment. Used by NEST.

* cam_tf.py - transceiver function which converts images acquired by the robot into input usable by NEST.

* husky_world.sdf - simulation world file in `Simulation Description Format <http://sdformat.org/>`__. Used by Gazebo.

* mot_tf.py - transceiver function that converts output from NEST into movement commands.

* simulation_config.json - simulation configuration file. An explanation of the simulation configuration can be found :ref:`here <doxid-simulation_configuration>`.





.. _doxid-running_example_exp_1getting_started_experiment_husky_pf:

Husky braitenberg with preprocessing function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This experiment is a modification of the previous one and shows how to use one of the tools provided by the NRP - the :ref:`preprocessing function <doxid-preprocessing_function>`. Preprocessing functions are called before transceiver functions. The use of preprocessing functions is intended to increase computational efficiency when several transceiver functions would otherwise carry out multiple times the same operations on the same data.

The preprocessing function used in the example converts RGB images received from Gazebo into grayscale images. The processed images are passed to one of the transceiver functions (``cam_tf.py``) and can be displayed after changing ``False`` to ``True`` in the following piece of code:

.. ref-code-block:: cpp

	# Set to True to display camera image data and pause for 10 s
	if False and not camera.isEmpty():
	    img = Image.fromarray(np.array(processed.data["grayscale"]))
	    img.show()
	    time.sleep(10)

The code of the preprocessing function is located in ``grayscale.py`` file.

To run the example, switch to the proper directory and run the NRP simulation executable:

.. ref-code-block:: cpp

	cd examples/husky_braitenberg_with_preprocessing
	NRPCoreSim -c simulation_config.json -p "NRPGazeboGrpcEngine.so,NRPNestJSONEngine.so"





.. _doxid-running_example_exp_1getting_started_experiment_husky_nest_server:

Husky braitenberg with NEST server
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This example is again a modification of the husky braitenberg experiment, in this case using a client-only engine that connects to an instance of `NEST server <https://pypi.org/project/nest-server/>`__. To load the engine, its library must be specified on the command line when launching the NRP executable:

.. ref-code-block:: cpp

	cd examples/husky_braitenberg_nest_server
	NRPCoreSim -c simulation_config.json -p "NRPGazeboGrpcEngine.so,NRPNestServerEngine.so"

This will also start nest-server in a separate process.





.. _doxid-running_example_exp_1getting_started_experiment_tf_exchange:

DataPack Exchange using the Python JSON Engine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This simple example shows two instances of :ref:`Python JSON Engine <doxid-python_json_engine>` exchanging data as datapacks. Data exchange between the simulators is done with the help of :ref:`Transceiver Functions <doxid-transceiver_function>` (TFs) and :ref:`datapacks <doxid-datapacks>`.

The PythonJSONEngine implements a Python class, *EngineScript*, the methods *initialize*, *runLoop* and *shutdown* of which can be overriden to execute any arbitrary piece of Python code synchronously in a NRP experiment. This opens the possibility of easily integrating any simulator with a Python API in an experiment (e.g. OpenSim, OpenAI Gym, etc.). Under the hood, *EngineScript* manages :ref:`DataPack <doxid-class_data_pack>` I/O operations with the Simulation Loop.

In the example, which can be found in *examples/tf_exchange* folder, an engine defined in *engine_1.py* file registers a datapack of type :ref:`JsonDataPack <doxid-datapacks_1datapacks_json>` with id "datapack1". The datapack stores a dictionary with the current simulation time in the engine and the number of simulation steps the engine has already advanced. Then a TF defined in *tf_1.py* gets this datapack and relays it to a second engine defined in *engine_2.py*, which simply prints out its data.

:ref:`Python JSON Engine <doxid-python_json_engine>` only supports *JsonDataPack* datapack type, which stores a wraps a JSON object in an attribute *data*. Therefore it allows to send any kind of data between Engines and TFs with the only constraint of it being JSON serializable. *JsonDataPack* is defined in the *nrp_core.nrp_json* module from which it can be imported. Eg:

.. ref-code-block:: cpp

	from nrp_core.data.nrp_json import JsonDataPack

To launch the example, just execute:

.. ref-code-block:: cpp

	cd examples/tf_exchange
	NRPCoreSim -c simulation_config.json

In this case, no additional plugin needs to be specified.





.. _doxid-running_example_exp_1getting_started_experiment_OpenSIm_Arm26:

OpenSim with dummy controller for Arm26 model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This experiment implements a dummy controller for a simple OpenSim model of the human arm (arm26, see `https://simtk-confluence.stanford.edu:8443/display/OpenSim/Musculoskeletal+Models <https://simtk-confluence.stanford.edu:8443/display/OpenSim/Musculoskeletal+Models>`__) in an OpenSim scene.

This experiment uses two kinds of so-called engines (:ref:`NRPPythonJSONEngine <doxid-python_json_engine>` and :ref:`NRPOpenSimEngine <doxid-opensim_engine>`), and illustrates how the NRP manages execution of the OpenSim simulation and synchronization with other engines. The communication between OpenSim and the python controller is done with the help of :ref:`transceiver functions <doxid-transceiver_function>` and :ref:`engine datapacks <doxid-datapacks>`. In short, the transceiver functions allow translating the I/O data between different platforms, while the datapacks serve as generic containers for the data.

To run the example, switch to the experiment directory and run the NRP simulation executable:

.. ref-code-block:: cpp

	cd examples/opensim_control
	NRPCoreSim -c simulation_config.json -p "NRPOpenSimEngine.so"

This will start the OpenSim simulation with proper visualization.

Here is a short description of all files that are located in the example directory:

* Folder “arm26” – include the simulation file of the OpenSim scene

* server.py - contains the setup of the OpenSim simulation, including initialize, runLoop, reset, and shutdown

* ``client.py`` - contains the setup of the python controller

* send_cmd.py - transceiver function that converts output from the Python controller into simulation commands

* rec_joints.py - transceiver function which converts feedback muscle data from OpenSim

* simulation_config.json - simulation configuration file. An explanation of the simulation configuration can be found :ref:`here <doxid-simulation_configuration>`


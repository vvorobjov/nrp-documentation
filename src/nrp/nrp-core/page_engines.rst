.. index:: pair: page; Engines
.. _doxid-engines:

Engines
=======

Engines are a core aspect of the NRP-core framework. They run the actual simulation software (which can be comprised of any number of heterogeneous modules), with the Simulation Loop and TransceiverFunctions merely being a way to synchronize and exchange data between them. The data exchange is carried out through an **engine client** (see paragraph below). An Engine can run any type of software, from physics engines to brain simulators. The only requirement is that they should be able to manage progressing through time with fixed-duration time steps.

From the NRP-core perspective, the core component of the engine is its communication interface, which enables it to communicate with the Simulation Loop. Different engine types can have different communication protocols. Nevertheless, all protocols are envisioned as a server-client architecture, with the Engine server running as a separate process, and a client running inside the Simulation Loop process. As such, all :ref:`EngineClients <doxid-class_engine_client>` must at least support the following functionalities:

* LaunchEngine: A function to launch the engine process. This will usually in some way use the :ref:`ProcessLauncherInterface <doxid-class_process_launcher_interface>`

* Initialize: A function that initializes the engine after launch

* Reset: A function which resets the Engine simulation or control process

* RunLoopStepAsync: A function that will advance the engine by a given time step

* UpdateDataPacksFromEngine: A function to retrieve :ref:`DataPack <doxid-class_data_pack>` data from the Engine

* SendDataPacksToEngine: A function to send :ref:`DataPack <doxid-class_data_pack>` data to the Engine

* Shutdown: A function that gracefully stops the Engine

This :ref:`page <doxid-nrp_engines>` contains a list of currently supported Engines.

The architecture has been designed in a modular way that allows to implement new engines with relative easiness. In :ref:`this page <doxid-tutorial_engine_creation>` can be found a tutorial on how to implement a new engine from scratch. An easier approach is to base the implementation on one of the already implemented communication protocol. Tutorials are available teaching how to accomplish that using gRPC and JSON over REST.



.. _doxid-engines_1Python_json_engine_section:

Python JSON Engine
~~~~~~~~~~~~~~~~~~

Most of the engine implementations distributed alongside NRP-core are bound to a specific simulator, eg. NEST or Gazebo. There is one important exception however: the :ref:`Python JSON Engine <doxid-python_json_engine>`.

This versatile engine allows one to execute a user-defined Python script as an engine server, thus ensuring synchronization and enabling datapack data transfer with the Simulation Loop process. It can be used as a quick way of integrating a not-yet supported simulator in a NRP-core experiment. In :ref:`this page <doxid-python_json_engine>`, interested users will find more information on how to use it.





.. _doxid-engines_1engine_launchers:

Engine Launchers
~~~~~~~~~~~~~~~~

An EngineLauncher is in charge of properly launching an engine using a given :ref:`ProcessLauncher <doxid-class_process_launcher>`. :ref:`EngineClient::EngineLauncher <doxid-class_engine_client_1_1_engine_launcher>` is provided for this purpose and can be used with any new :ref:`EngineClient <doxid-class_engine_client>`.

.. ref-code-block:: cpp

	// Define the EngineLauncher.
	using NewEngineLauncher = NewEngineClient::EngineLauncher<NewEngineConfigConst::EngineType>;

A new Engine library can use then ``NewEngineLauncher`` to make it plugin compatible. Look under :ref:`plugin system <doxid-plugin_system>` for additional details.

Note that we assign this EngineLauncher the name specified in NewEngineConfigConst::EngineType. Afterwards, a user can select this engine in the main simulation configuration file by setting as EngineType parameter this value. For details about setting up a simulation configuration file, look :ref:`here <doxid-simulation_configuration>`.

Should an engine require more complex startup routines, consider overriding :ref:`EngineClientInterface::launchEngine() <doxid-class_engine_client_interface_1a42dd02dc80abcc1f48dccf9da0ce2f0c>` in the new :ref:`EngineClient <doxid-class_engine_client>` implementation. Do not modify the default EngineLauncher, as its only purpose is to construct the Engine class and then call the above function.


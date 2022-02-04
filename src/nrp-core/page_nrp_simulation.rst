.. index:: pair: page; NRPCoreSim
.. _doxid-nrp_simulation:

NRPCoreSim
==========

This is the main NRP-core executable. It can be used to run an experiment locally. With it, a simulation is configured, a Simulation Loop is initialized, and a simulation is run. The process is divided into the following steps:

* Parse input parameters:
  
  * If a '-h' or ' help' is added to the simulation, only print the help text, then exit
  
  * '-c': configuration file name, it is expected to be in the same folder NRPCoreSim is executed from
  
  * '-p': Additional engine plugins to load. The expected value is a list of :ref:`engine plugin <doxid-plugin_system>`.so libraries separated by commas withouht spaces and between quotation marks. By default only :ref:`Python JSON Engine <doxid-python_json_engine>` is loaded

* Initialize the Python interpreter for TransceiverFunctions

* Setup the process launchers

* Load engine launchers:
  
  * Start the :ref:`PluginManager <doxid-class_plugin_manager>`
  
  * It will load all engine plugins defined as default on cmake configuration
  
  * Additionally, it will read the input parameter '-p' and load those engine plugins requested by the user
  
  * Store all engines launchers contained in the loaded plugins in an :ref:`EngineLauncherManager <doxid-class_engine_launcher_manager>`

* Use input parameters to generate a new instance of :ref:`SimulationManager <doxid-class_simulation_manager>`. This will also launch all engine processes defined in the SimulationConfig passed to NRPCoreSim

* If a SimulationConfig file was given as an input parameter, initialize a Simulation Loop and run until timeout

To launch an experiment with NRPCoreSim, the user must specify the simulation configuration file and optionally a list of engine plugins to be loaded. Eg:

.. ref-code-block:: cpp

	NRPCoreSim -c simulation_config.json -p "NRPGazeboGrpcEngine.so,NRPNestJSONEngine.so,NRPNestServerEngine.so"


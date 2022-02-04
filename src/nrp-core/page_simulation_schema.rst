.. index:: pair: page; Simulation Configuration Schema
.. _doxid-simulation_schema:

Simulation Configuration Schema
===============================

The simulation schema contains all the necessary information to initialize and run an experiment, including details such as the engines involved in the experiment, active TransceiverFunctions, and engine timesteps. It defines the structure and parameters of the simulation configuration file to be created by the users for each of their experiments.



.. _doxid-simulation_schema_1simulation_schema_parameters:

Parameters
~~~~~~~~~~

===========================  ========================================================================================================================================  =======  =========  ========  =====  ======================  
Name                         Description                                                                                                                               Type     Default    Required  Array  Values                  
===========================  ========================================================================================================================================  =======  =========  ========  =====  ======================  
SimulationLoop               Type of simulation loop used in the experiment                                                                                            enum     "FTILoop"                   "FTILoop", "EventLoop"  
SimulationTimeout            Experiment Timeout (in seconds). It refers to simulation time                                                                             integer  0                                                   
SimulationTimestep           Time in seconds the simulation advances in each                                                                                           number   0.01                                                
ProcessLauncherType                                                                                                                                                    string   Basic                                               
EngineConfigs                Engines that will be started in the experiment                                                                                                                X                                        
                             Framework used to process and rely datapack data between engines. Available options are the TF framework (tf) and Computation Graph (cg)  enum     "tf"                        "tf", "cg"              
DataPackProcessingFunctions  Transceiver and Preprocessing functions that will be used in the experiment                                                                        []                   X                              
                             List of filenames defining the                                                                                                            string   []                   X                              
StartROSNode                 If true a ROS node is started by NRPCoreSim                                                                                               boolean  false                                               
EventLoopTimeout             Event loop timeout (in seconds). 0 means no timeout. If not specified 'SimulationTimeout' is used instead                                 integer  0                                                   
EventLoopTimestep            Time in seconds the event loop advances in each loop. If not specified 'SimulationTimestep' is used instead                               number   0.01                                                
===========================  ========================================================================================================================================  =======  =========  ========  =====  ======================





.. _doxid-simulation_schema_1simulation_schema_example:

Example
~~~~~~~

.. ref-code-block:: cpp

	{
	    "SimulationName": "tf_exchange",
	    "SimulationDescription": "Launch two python engines. Engine1 will offer the current time as a datapack. tf_1 will request said datapack and send it to Engine2. Engine2 will receive the time and display it",
	    "SimulationTimeout": 1,
	    "EngineConfigs": 
	    [
	        {
	            "EngineType": "python_json",
	            "EngineName": "python_1",
	            "PythonFileName": "engine_1.py"
	        },
	        {
	            "EngineType": "python_json",
	            "EngineName": "python_2",
	            "PythonFileName": "engine_2.py"
	        }
	    ],
	    "DataPackProcessingFunctions":
	    [
	        {
	            "Name": "tf_1",
	            "FileName": "tf_1.py"
	        }
	    ]
	}





.. _doxid-simulation_schema_1simulation_schema_schema:

Schema
~~~~~~

.. ref-code-block:: cpp

	{
	  "$schema": "http://json-schema.org/draft-07/schema#",
	  "title": "Simulation",
	  "description": "Simulation configuration schema. Specify an experiment using multiple engines and transceiver functions.",
	  "$id": "#Simulation",
	  "type": "object",
	  "properties" : {
	    "SimulationLoop" : {
	      "enum" : ["FTILoop", "EventLoop"],
	      "default": "FTILoop",
	      "description": "Type of simulation loop used in the experiment"
	    },
	    "SimulationTimeout" : {
	      "type" : "integer",
	      "default": 0,
	      "description": "Experiment Timeout (in seconds). It refers to simulation time."
	    },
	    "SimulationTimestep" : {
	      "type" : "number",
	      "default": 0.01,
	      "description": "Time in seconds the simulation advances in each Simulation Loop. It refers to simulation time."
	    },
	    "DataPackProcessor" : {
	      "type" : "string",
	      "enum" :  ["tf", "cg"],
	      "default": "tf",
	      "description": "Framework used to process and rely datapack data between engines. Available options are the TF framework (tf) and Computation Graph (cg)"
	    },
	    "ProcessLauncherType" : {
	      "type" : "string",
	      "default": "Basic",
	      "description": "ProcessLauncher type to be used for launching engine processes"
	    },
	    "EngineConfigs" : {
	      "type" : "array",
	      "items": {"$ref": "https://neurorobotics.net/engines/engine_base.json#EngineBase"},
	      "description": "Engines that will be started in the experiment"
	    },
	    "DataPackProcessingFunctions" : {
	      "type" : "array",
	      "items": {"$ref": "https://neurorobotics.net/transceiver_function.json#TransceiverFunction"},
	      "description": "Transceiver and Preprocessing functions that will be used in the experiment"
	    },
	    "ComputationalGraph" : {
	      "type" : "array",
	      "items": "string",
	      "description": "List of filenames defining the ComputationalGraph that will be used in the experiment"
	    },
	    "StartROSNode" : {
	      "type": "boolean",
	      "default": false,
	      "description": "If true a ROS node is started by NRPCoreSim"
	    },
	    "EventLoopTimeout" : {
	      "type" : "integer",
	      "description": "Event loop timeout (in seconds). 0 means no timeout. If not specified 'SimulationTimeout' is used instead"
	    },
	    "EventLoopTimestep" : {
	      "type" : "number",
	      "description": "Time length (in seconds) of each loop, i.e it is the inverse of the Event Loop frequency. If not specified 'SimulationTimestep' is used instead"
	    }
	  },
	  "required": []
	}


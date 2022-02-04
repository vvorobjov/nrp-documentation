.. index:: pair: page; EngineBase Schema
.. _doxid-engine_base_schema:

EngineBase Schema
=================

The EngineConfigs array in the simulation configuration file describes the setup of the engines that will participate in the experiment. As different types of engines require different configuration parameters, each type have a dedicated schema. For a list of the available engines and their configuration parameters see :ref:`Engine implementations shipped with NRP-core <doxid-nrp_engines>`.

Regardless of the chosen engine type, there is a set of parameters common to every engine. These are defined in the EngineBase schema presented in this page. From the list of EngineBase parameters, several deserve specific explanations, which are given in a section :ref:`below <doxid-engine_base_schema_1specific_parameters>`.



.. _doxid-engine_base_schema_1engine_base_schema_parameters:

Parameters
~~~~~~~~~~

=====================  ===================================================================================================================================================  ======  =======  ========  =====  
Name                   Description                                                                                                                                          Type    Default  Required  Array  
=====================  ===================================================================================================================================================  ======  =======  ========  =====  
EngineName             Name of the engine                                                                                                                                   string           X                
EngineType             Engine type. Used by                                                                                                                                 string           X                
EngineProcCmd          Engine Process Launch command                                                                                                                        string                            
EngineProcStartParams  Engine Process Start Parameters                                                                                                                      string  []                 X      
EngineEnvParams        Engine Process Environment Parameters                                                                                                                string  []                 X      
EngineLaunchCommand                                                                                                                                                         string                            
EngineTimestep         Engine Timestep in seconds                                                                                                                           number  0.01                      
EngineCommandTimeout   Engine Timeout (in seconds). It tells how long to wait for the completion of the engine runStep. 0 or negative values are interpreted as no timeout  number  0.0                       
=====================  ===================================================================================================================================================  ======  =======  ========  =====





.. _doxid-engine_base_schema_1engine_base_schema_example:

Example
~~~~~~~

.. ref-code-block:: cpp

	{
	    "EngineType": "python_json",
	    "EngineName": "python_1",
	    "PythonFileName": "engine_1.py"
	}





.. _doxid-engine_base_schema_1engine_base_schema_schema:

Schema
~~~~~~

.. ref-code-block:: cpp

	{
	  "$schema": "http://json-schema.org/draft-07/schema#",
	  "title": "Engine Base",
	  "description": "Base configuration schema which all engine configurations inherit from",
	  "$id": "#EngineBase",
	  "type": "object",
	  "properties" : {
	    "EngineName" : {
	      "type" : "string",
	      "description": "Name of the engine"
	    },
	    "EngineType" : {
	      "type" : "string",
	      "description": "Engine type. Used by EngineLauncherManager to select the correct engine launcher"
	    },
	    "EngineProcCmd" : {
	      "type" : "string",
	      "description": "Engine Process Launch command"
	    },
	    "EngineProcStartParams" : {
	      "type" : "array",
	      "items": {"type" : "string"},
	      "description": "Engine Process Start Parameters"
	    },
	    "EngineEnvParams" : {
	      "type" : "array",
	      "items": {"type" : "string"},
	      "description": "Engine Process Environment Parameters"
	    },
	    "EngineLaunchCommand" : {
	      "type" : "string",
	      "default": "BasicFork",
	      "description": "LaunchCommand type that will be used to launch the engine process"
	    },
	    "EngineTimestep" : {
	      "type" : "number",
	      "default": 0.01,
	      "description": "Engine Timestep in seconds"
	    },
	    "EngineCommandTimeout" : {
	      "type" : "number",
	      "default": 0.0,
	      "description": "Engine Timeout (in seconds). It tells how long to wait for the completion of the engine runStep. 0 or negative values are interpreted as no timeout"
	    }
	  },
	  "required" : ["EngineName", "EngineType"]
	}





.. _doxid-engine_base_schema_1specific_parameters:

Additional notes on parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



.. _doxid-engine_base_schema_1engine_proc_command:

EngineProcCmd
-------------

This parameter sets the command that will be used to launch the engine. As mentioned in the :ref:`General notes about the use of JSON schema <doxid-json_schema>` section, default values are allowed to be set either in the schema or in-code, by using the ``json_utils::set_default`` function. In the later case, the parameter should be set as not required and should not have a default parameter. This is the case of ``EngineProcCmd``.

In order to allow for flexibility in the setting of ``EngineProcCmd``, its default value is set in-code for every engine type. This means that every new engine implementation should set a default value for ``EngineProcCmd`` in-code. For example, in the case of the :ref:`NEST JSON Engine <doxid-nest_json>`, ``EngineProcCmd`` is set to ``NRP_NEST_EXECUTABLE_PATH``, which is an environment variable pointing to nest-simulator executable.

In any case, if a value for ``EngineProcCmd`` is set in the configuration file, it will be used for launching the engine instead of the default value.





.. _doxid-engine_base_schema_1engine_proc_start_params:

EngineProcStartParams
---------------------

This parameter contains an array of strings that will be added as arguments to the Engine process launch command. In order to add flexibility in the formatting on these arguments, :ref:`EngineClient <doxid-class_engine_client>` has a dedicated method, :ref:`EngineClient::engineProcStartParams <doxid-class_engine_client_interface_1a6747137f2b551040adca807e6df38a59>`, in which the ``EngineProcStartParams`` parameter is read from the configuration file and formatted appropriately. This is a virtual method that **every new engine implementation should implement**.





.. _doxid-engine_base_schema_1engine_env_params:

EngineEnvParams
---------------

This case is very similar to the previous one. This parameter contains the environment variables that will be passed along with the Engine process launch command when starting the Engine process. There is a virtual :ref:`EngineClient::engineProcEnvParams <doxid-class_engine_client_interface_1ac3bf04a627785a082fbe83a5aa004227>` method that allows to format the environment variables appropriately and that **every new engine implementation should implement**.





.. _doxid-engine_base_schema_1additional_engine_configuration:

Additional Configuration Parameters
-----------------------------------

Engine implementations may need additional in-code configuration parameters which are not present in their schemas. The convention adopted in the architecture is to store these parameters in a struct with the name ``EngineTypeConfigConst``. For example, in the case of gazebo_json_engine exists a :ref:`GazeboJSONConfigConst <doxid-struct_gazebo_j_s_o_n_config_const>` :

.. ref-code-block:: cpp

	struct :ref:`GazeboJSONConfigConst <doxid-struct_gazebo_j_s_o_n_config_const>`
	{
	    static constexpr char :ref:`EngineType <doxid-struct_gazebo_j_s_o_n_config_const_1a8712a9c3c63ec96a486092b89e2a6a9a>`[] = "gazebo_json";
	    static constexpr char :ref:`EngineSchema <doxid-struct_gazebo_j_s_o_n_config_const_1aebf5499573d3a92aabec1a669741843c>`[] = "https://neurorobotics.net/engines/engines_gazebo.json#/engine_gazebo_json";
	
	    static constexpr std::string_view :ref:`GazeboPluginArg <doxid-struct_gazebo_j_s_o_n_config_const_1abdcff1882e2db6aef6b11ad3424c6de0>` = "-s";
	
	    static constexpr std::string_view :ref:`GazeboRNGSeedArg <doxid-struct_gazebo_j_s_o_n_config_const_1a84e800c1318b4de50c2e4a18b67ec89a>` = "--seed";
	};

Two parameters are mandatory for every engine implementation:

* ``EngineType`` : which is used to configure the :ref:`engine launcher <doxid-engines_1engine_launchers>` corresponding to this type of engine

* ``EngineSchema`` : which contains the path to the schema that will be used to validate an engine configuration for this specific type of engine. The engine configuration is passed to the :ref:`EngineClient <doxid-class_engine_client>` constructor as a `nlohmann::json <https://github.com/nlohmann/json>`__ object and validated against the specified schema in the same constructor. If the validation step fails the experiment will end with an exception.


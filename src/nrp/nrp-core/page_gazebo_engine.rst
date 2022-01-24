.. index:: pair: page; Gazebo Engine
.. _doxid-gazebo_engine:

Gazebo Engine
=============

This is an engine implementation that integrates the `Gazebo physics simulator <http://gazebosim.org/>`__ in NRP-core: the `gzserver executable <http://manpages.ubuntu.com/manpages/focal/man1/gzserver.1.html>`__ is running inside the gazebo engine server process.

The integration of gzserver in NRP-core is implemented through gazebo plugins, which must be used in the gazebo simulation sdf file in order to register the engine with the :ref:`SimulationManager <doxid-class_simulation_manager>` and setup datapack communication.

Two implementations of the Gazebo engine are provided. One is based on :ref:`JSON over REST <doxid-engine_comm_1engine_json>` and another on :ref:`Protobuf over gRPC <doxid-engine_comm_1engine_grpc>`. The latter performs much better and it is recommended. The former is provided for situations in which gRPC may not be available. The gRPC implementation uses protobuf objects to encapsulate data exchanged between the Engine and TFs, whereas the JSON implementation uses nlohmann::json objects. Besides from this fact, both engines are very similar both in their configuration and behavior. The rest of the documentation below is implicitely referred to the gRPC implementation even though in most cases the JSON implementation shows no differences.

A description of the implemented **gazebo plugins** can be found :ref:`here <doxid-gazebo_plugins>`, and a description of the **datapacks** supported off-the-shelf by the gazebo engine can be found :ref:`here <doxid-gazebo_datapacks>`



.. _doxid-gazebo_engine_1engine_gazebo_config_section:

Engine Configuration Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This Engine type parameters are defined in GazeboGRPCEngine schema (listed :ref:`here <doxid-gazebo_engine_1engine_gazebo_schema>`), which in turn is based on :ref:`EngineBase <doxid-engine_base_schema>` and :ref:`EngineGRPC <doxid-engine_comm_1engine_comm_protocols_schema>` schemas and thus inherits all parameters from them.

To use the Gazebo engine in an experiment, set ``EngineType`` to **"gazebo_grpc"**.

* Parameters inherited from :ref:`EngineBase <doxid-engine_base_schema>` schema:

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

* Parameters inherited from the :ref:`EngineGRPC <doxid-engine_comm_1engine_grpc>` schema:

=============  ===============================================================================================  ======  ==============  ========  =====  
Name           Description                                                                                      Type    Default         Required  Array  
=============  ===============================================================================================  ======  ==============  ========  =====  
ServerAddress  gRPC Server address. Should this address already be in use, simulation initialization will fail  string  localhost:9004                   
=============  ===============================================================================================  ======  ==============  ========  =====

* Parameters specific to this engine type:

===============  ==============================================================================================================================  =======  =======  ========  =====  
Name             Description                                                                                                                     Type     Default  Required  Array  
===============  ==============================================================================================================================  =======  =======  ========  =====  
GazeboWorldFile  Path to Gazebo SDF World file                                                                                                   string            X                
GazeboPlugins    Additional system plugins that should be loaded on startup                                                                      string   []                 X      
GazeboRNGSeed    Seed parameters passed to gzserver start command                                                                                integer  0                         
WorldLoadTime    Maximum time (in seconds) to wait for the NRPCommunicationPlugin to load the world sdf file. 0 means it will wait indefinitely  integer  20                        
===============  ==============================================================================================================================  =======  =======  ========  =====





.. _doxid-gazebo_engine_1engine_gazebo_schema:

Schema
~~~~~~

As explained above, the schema used by the Gazebo engine inherits from :ref:`EngineBase <doxid-engine_base_schema>` and :ref:`EngineGRPC <doxid-engine_comm_1engine_comm_protocols_schema>` schemas. A complete schema for the configuration of this engine is given below:

.. ref-code-block:: cpp

	{"engine_gazebo_base" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Gazebo Base",
	    "description": "Gazebo Base Engine configuration schema. Configuration for all gazebo engine implementations inherit from this one",
	    "$id": "#GazeboBase",
	    "properties" : {
	      "GazeboWorldFile": {
	        "type": "string",
	        "description": "Path to Gazebo SDF World file"
	      },
	      "GazeboPlugins": {
	        "type": "array",
	        "items": {"type": "string"},
	        "description": "Additional system plugins that should be loaded on startup"
	      },
	      "GazeboRNGSeed": {
	        "type": "integer",
	        "default": 0,
	        "description": "Seed parameters passed to gzserver start command"
	      },
	      "WorldLoadTime": {
	        "type": "integer",
	        "default": 20,
	        "description": "Maximum time (in seconds) to wait for the NRPCommunicatioPlugin to load the world sdf file. 0 means it will wait indefinitely"
	      }
	    },
	    "required": ["GazeboWorldFile"]
	  },
	  "engine_gazebo_grpc" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Gazebo Grpc Engine",
	    "description": "Gazebo Grpc Engine",
	    "$id": "#GazeboGRPCEngine",
	    "allOf": [
	      { "$ref": "https://neurorobotics.net/engines/engine_comm_protocols.json#/engine_grpc" },
	      { "$ref": "#/engine_gazebo_base" },
	      {
	        "properties": {
	          "EngineType": { "enum": ["gazebo_grpc"] },
	          "EngineProcCmd": { "default": "/usr/bin/gzserver" }
	        }
	      }
	    ]
	  },
	  "engine_gazebo_json" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Gazebo Json Engine",
	    "description": "Gazebo Json Engine",
	    "$id": "#GazeboJSONEngine",
	    "allOf": [
	      { "$ref": "https://neurorobotics.net/engines/engine_comm_protocols.json#/engine_json" },
	      { "$ref": "#/engine_gazebo_base" },
	      {
	        "properties": {
	          "EngineType": { "enum": ["gazebo_json"] },
	          "EngineProcCmd": { "default": "/usr/bin/gzserver" }
	        }
	      }
	    ]
	  }
	}

.. toctree::
	:hidden:

	page_gazebo_datapacks.rst
	page_gazebo_plugins.rst

.. rubric:: Related Pages:

|	:doc:`page_gazebo_datapacks`
|	:doc:`page_gazebo_plugins`



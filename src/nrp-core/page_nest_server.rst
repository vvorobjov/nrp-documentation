.. index:: pair: page; NEST Server Engine
.. _doxid-nest_server:

NEST Server Engine
==================

This engine allows to use out of the box the `nest-server <https://pypi.org/project/nest-server/>`__ application that comes with NEST 3 together with NRP-core. In this case the Engine server is nest-server itself without modifications. Thus the engine implementation is reduced to the engine client, which translates requests for advancing the simulation and sending or getting datapack data to nest-server using its REST API.

From a user point of view, this engine is very similar to :ref:`NEST JSON Engine <doxid-nest_json>`. At initialization time, it loads a NEST simulation model described in a python script. At run time, it takes care of advancing the simulation when requested and of handling datapack data transfer.

The main difference is that in this case datapacks can't be registered from within the aforementioned Python script, as it is done in the case of NestJsonEngine. Instead, users rely on the mechanism provided by nest-server to access population status. A dictionary with name **populations** can be defined in the python script. Afterwards the keys of this dictionary can be accessed as JsonDataPacks in TransceiverFunctions. The limitations on the objects that can be used as values in this dictionary will be those imposed by nest-server.



.. _doxid-nest_server_1nest_server_datapacks:

DataPacks
~~~~~~~~~

This engine supports a unique datapack type: JsonDataPack. It contains the status of a set of NEST objects as a list of dictionaries.

The way to register a JsonDataPack datapack is adding it to the **populations** dictionary in the initialization Python script. Afterwards, the status of the value to this key in the dictionary can be accessed or set in TransceiverFunctions.

The experiment located in ``examples/husky_braitenberg_nest_server`` shows how the NEST Server Engine can be configured in NRP-core. It can be observed that it is almost indistinguishable from its :ref:`NEST JSON Engine <doxid-nest_json>` version, which can be found in ``examples/husky_braitenberg``. The only difference is that datapacks are registered through the populations dictionary:

.. ref-code-block:: cpp

	populations = {
	        'circuit' : population,
	        'lpg' : lpg,
	        'rpg' : rpg,
	        'gpg' : gpg,
	        'actors' : leaky_cells
	}

=========  =======================================================  ============  ==============  
Attribute  Description                                              Python Type   C type          
=========  =======================================================  ============  ==============  
data       data contained in the datapack as a NlohmannJson object  NlohmannJson  nlohmann::json  
=========  =======================================================  ============  ==============





.. _doxid-nest_server_1nest_server_configuration:

Engine Configuration Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This Engine type parameters are defined in the NestServerEngine schema (listed :ref:`here <doxid-nest_server_1nest_server_schema>`), which in turn is based on :ref:`EngineBase <doxid-engine_base_schema>` thus inherits all parameters from them.

To use the NEST server engine in an experiment, set ``EngineType`` to **"nest_server"**.

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

* Parameters specific to this engine type:

================  ============================================================================  =======  =====================================  ========  =====  
Name              Description                                                                   Type     Default                                Required  Array  
================  ============================================================================  =======  =====================================  ========  =====  
NestInitFileName  Path to the Python script that sets up the neural network for the simulation  string                                          X                
NestRNGSeed       Nest RNG seed                                                                 integer  0                                                       
NestServerHost    Nest Server Host                                                              string   localhost                                               
NestServerPort    Nest Server                                                                   integer  first unbound port starting from 5000                   
MPIProcs          Number of MPI processes used in the NEST simulation                           integer  1                                                       
================  ============================================================================  =======  =====================================  ========  =====





.. _doxid-nest_server_1nest_server_schema:

Schema
~~~~~~

As explained above, the schema used by the NEST engine inherits from :ref:`EngineBase <doxid-engine_base_schema>` schema. A complete schema for the configuration of this engine is given below:

.. ref-code-block:: cpp

	{"engine_nest_base" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Nest Base",
	    "description": "Nest Base Engine configuration schema. Configuration for all nest engine implementations inherit from this one",
	    "$id": "#NestBase",
	    "properties" : {
	      "NestInitFileName": {
	        "type": "string",
	        "description": "Path to the python script that sets up the neural network for the simulation"
	      },
	      "NestRNGSeed": {
	        "type": "integer",
	        "default": 0,
	        "description": "Nest RNG seed"
	      }
	    },
	    "required": ["NestInitFileName"]
	  },
	  "engine_nest_json" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Nest Json Engine",
	    "description": "Nest Json Engine",
	    "$id": "#NestJSONEngine",
	    "allOf": [
	      { "$ref": "https://neurorobotics.net/engines/engine_comm_protocols.json#/engine_json" },
	      { "$ref": "#/engine_nest_base" },
	      {
	        "properties": {
	          "EngineType": { "enum": ["nest_json"] }
	        }
	      }
	    ]
	  },
	  "engine_nest_server" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Nest Server Engine",
	    "description": "Nest Server Engine",
	    "$id": "#NestServerEngine",
	    "allOf": [
	      { "$ref": "https://neurorobotics.net/engines/engine_base.json#EngineBase" },
	      { "$ref": "#/engine_nest_base" },
	      {
	        "properties": {
	          "EngineType": { "enum": ["nest_server"] },
	          "NestServerHost" : {
	            "type": "string",
	            "default": "localhost",
	            "description": "Nest Server Host"
	          },
	          "NestServerPort": {
	            "type": "integer",
	            "description": "Nest Server Port"
	          },
	          "MPIProcs": {
	            "type": "integer",
	            "default": 1,
	            "description": "Number of MPI processes used in the NEST simulation"
	          }
	        }
	      }
	    ]
	  }
	}


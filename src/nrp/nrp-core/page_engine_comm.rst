.. index:: pair: page; Engine Communication Protocols
.. _doxid-engine_comm:

Engine Communication Protocols
==============================

Final engine implementations can be based on one of the engine communication protocol (ECP) provided with NRP-core. These are intermediate engine implementations (ie. can't be instantiated, nor directly used in an experiment) that implement engine client / server communication and datapack de-/serialization for a given communication protocol or middleware. Therefore, engine implementations based thereon can focus on the aspects specific to the functionality that the engine is meant to provide.

A tutorial on how to create new engine implementations based on the provided ECPs can be found :ref:`here <doxid-engine_creation_template>`.

The ECPs provided with NRP-core are described below.



.. _doxid-engine_comm_1engine_json:

JSON over REST
~~~~~~~~~~~~~~

This ECP uses an HTTP REST Server as the base for the engine server. The client can then use REST calls to communicate. All communications will be de-/serialized using JSON.

The server side of the engine is implemented in :ref:`EngineJSONServer <doxid-class_engine_j_s_o_n_server>` and the client in :ref:`EngineJSONNRPClient <doxid-class_engine_j_s_o_n_n_r_p_client>`. Final engine implementations can inherit from these two classes. In order to handle datapack I/O operations in the engine server, concrete datapack controllers can be based on the :ref:`DataPackController <doxid-class_data_pack_controller>` class.

To help create servers, :ref:`EngineJSONOptsParser <doxid-class_engine_j_s_o_n_opts_parser>` can be used to parse start parameters and extract relevant information. Additionally, the :ref:`SimulationManager <doxid-class_simulation_manager>` will launch an instance of :ref:`EngineJSONRegistrationServer <doxid-class_engine_j_s_o_n_registration_server>` which can be used by EngineJSONServers to communicate its address to clients.



.. _doxid-engine_comm_1engine_json_config_section:

Engine Configuration Parameters
-------------------------------

This engine type parameters are defined in EngineJSON schema (listed :ref:`here <doxid-engine_comm_1engine_comm_protocols_schema>`), which in turn is based on :ref:`EngineBase <doxid-engine_base_schema>` schema and thus inherits all parameters from the latter.

* Parameters inherited from EngineBase schema:

=====================  ===================================================================================================================================================  ======  =======  ========  =====  
Name                   Description                                                                                                                                          Type    Default  Required  Array  
=====================  ===================================================================================================================================================  ======  =======  ========  =====  
EngineName             Name of the engine                                                                                                                                   string           X                
EngineType             Engine type. Used by                                                                                                                                 string           X                
EngineProcCmd          Command (executable) that will be run in the engine process                                                                                          string                            
EngineProcStartParams  Engine Process Start Parameters                                                                                                                      string  []                 X      
EngineEnvParams        Engine Process Environment Parameters                                                                                                                string  []                 X      
EngineLaunchCommand                                                                                                                                                         string                            
EngineTimestep         Engine Timestep in seconds                                                                                                                           number  0.01                      
EngineCommandTimeout   Engine Timeout (in seconds). It tells how long to wait for the completion of the engine runStep. 0 or negative values are interpreted as no timeout  number  0.0                       
=====================  ===================================================================================================================================================  ======  =======  ========  =====

* Parameters specific to this engine type:

=========================  ===========  ======  ==============  ========  =====  
Name                       Description  Type    Default         Required  Array  
=========================  ===========  ======  ==============  ========  =====  
ServerAddress                           string  localhost:9002                   
RegistrationServerAddress  Address      string  localhost:9001                   
=========================  ===========  ======  ==============  ========  =====







.. _doxid-engine_comm_1engine_grpc:

Protobuf over gRPC
~~~~~~~~~~~~~~~~~~

In this case a gRPC server is used for communication. DataPacks are serialized into protobuf format.



.. _doxid-engine_comm_1engine_grpc_config_section:

Engine Configuration Parameters
-------------------------------

This engine type parameters are defined in EngineGRPC schema (listed :ref:`here <doxid-engine_comm_1engine_comm_protocols_schema>`), which in turn is based on :ref:`EngineBase <doxid-engine_base_schema>` schema and thus inherits all parameters from the latter.

* Parameters inherited from EngineBase schema:

=====================  ===================================================================================================================================================  ======  =======  ========  =====  
Name                   Description                                                                                                                                          Type    Default  Required  Array  
=====================  ===================================================================================================================================================  ======  =======  ========  =====  
EngineName             Name of the engine                                                                                                                                   string           X                
EngineType             Engine type. Used by                                                                                                                                 string           X                
EngineProcCmd          Command (executable) that will be run in the engine process                                                                                          string                            
EngineProcStartParams  Engine Process Start Parameters                                                                                                                      string  []                 X      
EngineEnvParams        Engine Process Environment Parameters                                                                                                                string  []                 X      
EngineLaunchCommand                                                                                                                                                         string                            
EngineTimestep         Engine Timestep in seconds                                                                                                                           number  0.01                      
EngineCommandTimeout   Engine Timeout (in seconds). It tells how long to wait for the completion of the engine runStep. 0 or negative values are interpreted as no timeout  number  0.0                       
=====================  ===================================================================================================================================================  ======  =======  ========  =====

* Parameters specific to this engine type:

=============  ===============================================================================================  ======  ==============  ========  =====  
Name           Description                                                                                      Type    Default         Required  Array  
=============  ===============================================================================================  ======  ==============  ========  =====  
ServerAddress  gRPC Server address. Should this address already be in use, simulation initialization will fail  string  localhost:9004                   
=============  ===============================================================================================  ======  ==============  ========  =====







.. _doxid-engine_comm_1engine_comm_protocols_schema:

Schema
~~~~~~

Inherits from :ref:`EngineBase <doxid-engine_base_schema>` schema

.. ref-code-block:: cpp

	{"engine_json" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Engine Json",
	    "description": "Base Json Engine configuration schema",
	    "$id": "#EngineJSON",
	    "allOf": [
	      { "$ref": "https://neurorobotics.net/engines/engine_base.json#EngineBase" },
	      {
	        "properties" : {
	          "ServerAddress": {
	            "type": "string",
	            "default": "localhost:9002",
	            "description": "Address from which the engine server sends/receives data"
	          },
	          "RegistrationServerAddress": {
	            "type": "string",
	            "default": "localhost:9001",
	            "description": "Address to which servers should register to"
	          }
	        }
	      }
	    ]
	  },
	  "engine_grpc" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Engine Grpc",
	    "description": "Base Grpc Engine configuration schema",
	    "$id": "#EngineGRPC",
	    "allOf": [
	      { "$ref": "https://neurorobotics.net/engines/engine_base.json#EngineBase" },
	      {
	        "properties" : {
	          "ServerAddress": {
	            "type": "string",
	            "default": "localhost:9004",
	            "description": "Address from which the engine server sends/receives data"
	          }
	        }
	      }
	    ]
	  }
	}


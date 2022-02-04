.. index:: pair: page; NEST JSON Engine
.. _doxid-nest_json:

NEST JSON Engine
================

This engine type enables the integration of the `NEST <https://www.nest-simulator.org/>`__ spiking neural network simulator in NRP-core. It is based on the :ref:`JSON over REST <doxid-engine_comm_1engine_json>` engine.

At initialization time, it loads a NEST simulation model described in a python script. At run time, it takes care of advancing the simulation when requested and of handling datapack data transfer.



.. _doxid-nest_json_1nest_json_datapacks:

DataPacks
~~~~~~~~~

This engine supports a unique datapack type: :ref:`JsonDataPack <doxid-datapacks_1datapacks_json>`. It contains the status of a set of neurons, detectors or synapsis as a dictionary or list of dictionaries. In fact, it can store the status of any NEST object which can be passed as argument to ``nest.GetStatus()`` and ``nest.SetStatus()`` functions.

nrp_core.engines.nest_json contains two functions that can be used to register datapacks with a NEST Json Engine:

* RegisterDataPack: takes as arguments a string indicating the datapack name and a NEST object which status will be linked to the datapack

* CreateDataPack: convenience function that internally calls nest.Create() and then :ref:`RegisterDataPack() <doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a29ef04a5068442acee81c1de9c94bdcc>`. Its first argument is the datapack name. The rest of the arguments the function is called with will be passed to nest.Create(). Both the datapack name and the return value of nest.Create() are then passed to :ref:`RegisterDataPack() <doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a29ef04a5068442acee81c1de9c94bdcc>`.

After a datapack is registered using the functions described above, a :ref:`NestEngineJSONDataPackController <doxid-class_nest_engine_j_s_o_n_data_pack_controller>` object is created to handle datapack I/O operations for this datapack in the Engine server. The NEST object passed to :ref:`RegisterDataPack() <doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a29ef04a5068442acee81c1de9c94bdcc>` is linked to this datapack controller. When the datapack is sent to the engine, it is passed to the controller which calls nest.SetStatus() with the datapack content. When the datapack is requested from the engine, the datapack controller calls nest.GetStatus() and stores the return value in the datapack data attribute.

In ``examples/nest_simple`` can be found a simple experiment showing the use of NEST JSON Engine. First, in the script ``nest_simple.py``, a NEST network is defined and two datapacks with names ``noise`` and ``voltage`` are registered:

.. ref-code-block:: cpp

	"""Init File. Imports nest and sets up a poisson generator, neuron, and voltmeter"""
	
	import nest
	import nest.voltage_trace
	from nrp_core.engines.nest_json import RegisterDataPack, CreateDataPack
	
	nest.set_verbosity("M_WARNING")
	nest.ResetKernel()
	
	noise = :ref:`CreateDataPack <doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a845e37097987f38a00e0921eeabb48d4>`("noise", "poisson_generator", 2)
	neuron = nest.Create("iaf_psc_alpha")
	voltmeter = nest.Create("voltmeter")
	:ref:`RegisterDataPack <doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a29ef04a5068442acee81c1de9c94bdcc>`('voltage', voltmeter)
	
	nest.Connect(noise, neuron, syn_spec={'weight': [[1.2, -1.0]], 'delay': 1.0})
	nest.Connect(voltmeter, neuron)
	
	# EOF

Then, in ``tf_1.py``, the ``voltage`` JsonDataPack is accessed and its content printed out. Finally it returns a ``noise`` datapack that will be used to set the status of its linked NEST object after it has been received by the engine server:

.. ref-code-block:: cpp

	from nrp_core import *
	from nrp_core.data.nrp_json import *
	
	from math import sin, cos
	
	sin_x = [abs(1000*sin(x)) for x in range(100)]
	cos_x = [abs(1000*cos(x)) for x in range(100)]
	n = 0
	print(sin_x)
	
	
	@:ref:`EngineDataPack <doxid-class_engine_data_pack>`(keyword='voltage', id=:ref:`DataPackIdentifier <doxid-struct_data_pack_identifier>`('voltage', 'nest'))
	@:ref:`TransceiverFunction <doxid-class_transceiver_function>`("nest")
	def transceiver_function(voltage):
	    # Read voltage
	    print(voltage)
	
	    # Set rate
	    global n
	    n = n+1 if n < 99 else 0
	
	    noise_datapack = :ref:`JsonDataPack <doxid-class_data_pack>`("noise", "nest")
	    noise_datapack.data.append({"rate": sin_x[n]})
	    noise_datapack.data.append({"rate": cos_x[n]})
	
	    return [ noise_datapack ]

=========  =======================================================  ============  ==============  
Attribute  Description                                              Python Type   C type          
=========  =======================================================  ============  ==============  
data       data contained in the datapack as a NlohmannJson object  NlohmannJson  nlohmann::json  
=========  =======================================================  ============  ==============





.. _doxid-nest_json_1nest_json_configuration:

Engine Configuration Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This Engine type parameters are defined in the NestJSONEngine schema (listed :ref:`here <doxid-nest_json_1nest_json_schema>`), which in turn is based on :ref:`EngineBase <doxid-engine_base_schema>` and :ref:`EngineJSON <doxid-engine_comm_1engine_comm_protocols_schema>` schemas and thus inherits all parameters from them.

To use the NEST JSON engine in an experiment, set ``EngineType`` to **"nest_json"**.

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

* Parameters inherited from :ref:`EngineJSON <doxid-engine_comm_1engine_json>` schema:

=========================  ===========  ======  ==============  ========  =====  
Name                       Description  Type    Default         Required  Array  
=========================  ===========  ======  ==============  ========  =====  
ServerAddress                           string  localhost:9002                   
RegistrationServerAddress  Address      string  localhost:9001                   
=========================  ===========  ======  ==============  ========  =====

* Parameters specific to this engine type:

================  ============================================================================  =======  =======  ========  =====  
Name              Description                                                                   Type     Default  Required  Array  
================  ============================================================================  =======  =======  ========  =====  
NestInitFileName  Path to the Python script that sets up the neural network for the simulation  string            X                
NestRNGSeed       Nest RNG seed                                                                 integer  0                         
================  ============================================================================  =======  =======  ========  =====





.. _doxid-nest_json_1nest_json_schema:

Schema
~~~~~~

As explained above, the schema used by the NEST JSON engine inherits from :ref:`EngineBase <doxid-engine_base_schema>` and :ref:`EngineJSON <doxid-engine_comm_1engine_comm_protocols_schema>` schemas. A complete schema for the configuration of this engine is given below:

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


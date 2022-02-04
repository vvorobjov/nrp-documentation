.. index:: pair: page; Python JSON Engine
.. _doxid-python_json_engine:

Python JSON Engine
==================

This versatile engine enables users to execute a user-defined python script as an engine server, thus ensuring synchronization and enabling datapack data transfer with the Simulation Loop process. It can be used to integrate any simulator with a Python API in a NRP-core experiment.

Engines based on PythonJSONEngine can be implemented as Python classes based on **EngineScript** (as in the example listed below). To register the engine with the :ref:`SimulationManager <doxid-class_simulation_manager>` in an experiment, the decorator **@RegisterEngine()** must be prepended to the class definition.



.. _doxid-python_json_engine_1python_json_engine_script:

EngineScript
~~~~~~~~~~~~

``EngineScript`` provides a base class from which custom Engines can inherit. The derived class must implement methods:

* ``initialize()`` : executed when the engine is initialized

* ``runLoop(timestep)`` : executed when the engine is requested to advance its simulation (from EngineClient::runLoopStep)

* ``shutdown()`` : executed when the engine is requested to shutdown

Optionally, the derived class can implement a ``reset()`` function. If it is implemented, it will be used for resetting the Engine. Otherwise the Engine is reset by calling ``shutdown()`` and ``initialize()`` sequentially.

Besides, ``EngineScript`` provides the following ready-to-use methods to handle datapack communication:

* ``_time()`` : returns the internal simulation time of the Engine.

* ``_getDataPack(datapack_name)`` : returns the latest value available of a datapack with name ``datapack_name``

* ``_setDataPack(datapack_name, data)`` : sets a new value for a datapack with name ``datapack_name``. ``data`` is always a Python dictionary.

* ``_registerDataPack(datapack_name)`` : registers a datapack with the engine. Just registered datapacks can be ``set`` and ``get``. Under the hood, registered datapacks are sent to the corresponding :ref:`EngineClient <doxid-class_engine_client>` upon request and their values updated when the :ref:`EngineClient <doxid-class_engine_client>` send them.

* ``_config`` : returns the engine configuration as a :ref:`JsonDataPack <doxid-datapacks_1datapacks_json>` object

Below is an example of a class inheriting from ``EngineScript``. The example is taken from the ``examples/tf_exchange`` experiment.

.. ref-code-block:: cpp

	"""Python Engine 1. Will get current engine time and make it accessible as a datapack"""
	
	from nrp_core.engines.python_json import EngineScript,RegisterEngine
	
	@RegisterEngine()
	class Script(EngineScript):
	    def initialize(self):
	        """Initialize datapack1 with time"""
	        print("Engine 1 is initializing. Registering datapack...")
	        self._registerDataPack("datapack1")
	        self._setDataPack("datapack1", { "time" : self._time.count(), "timestep": 0 })
	
	    def runLoop(self, timestep):
	        """Update datapack1 at every timestep"""
	        self._setDataPack("datapack1", { "time" : self._time.count(), "timestep": timestep.count() })
	        print("DataPack 1 data is " + str(self._getDataPack("datapack1")))
	
	    def shutdown(self):
	        print("Engine 1 is shutting down")
	
	    def reset(self):
	        print("Engine 1 is resetting")





.. _doxid-python_json_engine_1python_json_datapacks:

DataPacks
~~~~~~~~~

The Python JSON engine supports a unique datapack type: *JsonDataPack*, which can be used to transfer information between the engine and TFs. The data contained in this datapack can be any JSON-serializable Python object; that is, any object that can be decoded/encoded by `JSONDecoder/JSONEncoder <https://docs.python.org/3/library/json.html>`__. This data can be accessed in TransceiverFunctions from the datapack *data* attribute, as shown in this example TF (also taken from ``examples/tf_exchange``):

.. ref-code-block:: cpp

	from nrp_core import *
	from nrp_core.data.nrp_json import *
	
	@:ref:`EngineDataPack <doxid-class_engine_data_pack>`(keyword='datapack_python', id=:ref:`DataPackIdentifier <doxid-struct_data_pack_identifier>`('datapack1', 'python_1'))
	@:ref:`TransceiverFunction <doxid-class_transceiver_function>`("python_2")
	def transceiver_function(datapack_python):
	    rec_datapack1 = :ref:`JsonDataPack <doxid-class_data_pack>`("rec_datapack2", "python_2")
	    for k in datapack_python.data.keys():
	        rec_datapack1.data[k] = datapack_python.data[k]
	
	    return [ rec_datapack1 ]

=========  =======================================================  ============  ==============  
Attribute  Description                                              Python Type   C type          
=========  =======================================================  ============  ==============  
data       data contained in the datapack as a NlohmannJson object  NlohmannJson  nlohmann::json  
=========  =======================================================  ============  ==============





.. _doxid-python_json_engine_1python_json_configuration:

Engine Configuration Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This Engine type parameters are defined in the PythonJSONEngine schema (listed :ref:`here <doxid-python_json_engine_1python_json_schema>`), which in turn is based on :ref:`EngineBase <doxid-engine_base_schema>` and :ref:`EngineJSON <doxid-engine_comm_1engine_comm_protocols_schema>` schemas and thus inherits all parameters from them.

To use the Python JSON engine in an experiment, set ``EngineType`` to **"python_json"**.

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

==============  ==========================================================  ======  =======  ========  =====  
Name            Description                                                 Type    Default  Required  Array  
==============  ==========================================================  ======  =======  ========  =====  
PythonFileName  Path to the Python script containing the engine definition  string           X                
==============  ==========================================================  ======  =======  ========  =====





.. _doxid-python_json_engine_1python_json_schema:

Schema
~~~~~~

As explained above, the schema used by the PythonJSON engine inherits from :ref:`EngineBase <doxid-engine_base_schema>` and :ref:`EngineJSON <doxid-engine_comm_1engine_comm_protocols_schema>` schemas. A complete schema for the configuration of this engine is given below:

.. ref-code-block:: cpp

	{"python_base" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Python Json Engine Base",
	    "description": "Python Json Engine Base Configuration",
	    "$id": "#PythonJSONEngineBase",
	    "allOf": [
	      { "$ref": "https://neurorobotics.net/engines/engine_comm_protocols.json#/engine_json" },
	      {
	        "properties": {
	          "PythonFileName" : {
	            "type": "string",
	            "description": "Path to the python script containing the engine definition"
	          }
	        },
	        "required": ["PythonFileName"]
	      }
	    ]
	  },
	  "python_json" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Python Json Engine",
	    "description": "Python Json Engine Configuration",
	    "$id": "#PythonJSONEngine",
	    "allOf": [
	      { "$ref": "#/python_base" },
	      {
	        "properties": {
	          "EngineType": { "enum": ["python_json"] }
	        }
	      }
	    ]
	  },
	  "py_sim" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Python Simulation Engine",
	    "description": "Python Simulation Engine Configuration",
	    "$id": "#PySim",
	    "allOf": [
	      { "$ref": "#/python_base" },
	      {
	        "properties": {
	          "Simulator": {
	            "type": "string",
	            "default": "Opensim",
	            "description": "The simulator will be used"
	          },
	          "WorldFileName": {
	            "type": "string",
	            "description": "Path to the file of simulation world"
	          },
	          "Visualizer": {
	            "type": "boolean",
	            "default": false,
	            "description": "To show the simulation in visualizer or not"
	          }
	        },
	        "required": ["WorldFileName"]
	      }
	    ]
	  },
	  "opensim" : {
	    "$schema": "http://json-schema.org/draft-07/schema#",
	    "title": "Opensim Simulation Engine",
	    "description": "Opensim Simulation Engine Configuration",
	    "$id": "#OpenSimEngine",
	    "allOf": [
	      { "$ref": "#/py_sim" },
	      {
	        "properties": {
	          "EngineType": { "enum": ["opensim"] },
	          "Simulator": { "enum": ["Opensim"] }
	        }
	      }
	    ]
	  }
	}


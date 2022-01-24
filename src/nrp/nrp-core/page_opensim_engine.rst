.. index:: pair: page; OpenSim Engine
.. _doxid-opensim_engine:

OpenSim Engine
==============

This engine is based on the :ref:`Python JSON Engine <doxid-python_json_engine>` and exploits the `OpenSim Python API <https://simtk.org/api_docs/opensim/api_docs#>`__ to interface an Opensim simulation with NRP-core. It takes an Opensim model (stored in a \*.osim\* file) from the engine configuration and manages the synchronization and data exchange with other engines participating in the same experiment.

The engine is used in a manner very similar to the Python JSON Engine (please refer to this :ref:`guide <doxid-python_engine_guide>` for details on how to use it). Additionally, there is an example experiment in the folder *examples/opensim_control* that can be used as a reference for implementing experiments including the Opensim engine.

Similarly to the Python JSON Engine, the engine behavior in each experiment is implemented by subclassing a Python class *OpenSimEngineScript* and overriding the hook methods:

* ``initialize()`` : executed when the engine is initialized

* ``runLoop(timestep)`` : executed when the engine is requested to advance its simulation (from EngineClient::runLoopStep)

* ``shutdown()`` : executed when the engine is requested to shutdown

* ``reset()`` : executed when the engine is requested to reset. If it is not provided the Engine is reset by calling ``shutdown()`` and ``initialize()`` sequentially.

The interaction with the Opensim simulator is performed by an instance of the *SimulatorManager* class, which is stored in *OpenSimEngineScript* in the atribute *self.sim_manager*. *self.sim_manager* must be "manually" called from the *OpenSimEngineScript* subclass in order to advance, modify or get information from the Opensim simulation. As an example of this use, the script implementing the Opensim engine in *examples/opensim_control* is listed below:

.. ref-code-block:: cpp

	from nrp_core.engines.opensim import OpenSimEngineScript
	from nrp_core.engines.python_json import RegisterEngine
	
	# The API of Opensim is shown in the following link:
	# https://simtk.org/api_docs/opensim/api_docs
	
	@RegisterEngine()
	class Script(OpenSimEngineScript):
	    def initialize(self):
	        """Initialize datapack1 with time"""
	        print("Server Engine is initializing")
	        print("Registering datapack --> for sensors")
	        self._registerDataPack("joints")
	        self._setDataPack("joints", {"shoulder": 0, "elbow": 0})
	        self._registerDataPack("infos")
	        self._setDataPack("infos", {"time": 0})
	
	        # To set the force of muscles, in arm_26, they are:
	        # ['TRIlong', 'TRIlat', 'TRImed', 'BIClong', 'BICshort', 'BRA']
	        # The default color of muscle in the visualizer is blue.
	        # Once the force of a muscle is not the default value, 
	        # the color of the muscle will be changed. 
	        # Using this phenomenon, the controlled muscles can be found in the visualizer
	        # For example, if action=[0.5, 0.0, 0.0, 0.0, 0.0, 0.0], 
	        # the color of TRIlong will not be blue in shown screen
	        self.action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	        print("Registering datapack --> for actuators")
	        self._registerDataPack("control_cmd")
	
	    def runLoop(self, timestep):
	        # Receive control data from TF
	        self.action = self._getDataPack("control_cmd").get("act_list")
	        reset_falg = self._getDataPack("control_cmd").get("reset")
	
	        if reset_falg == 1:
	            self.reset()
	        else:
	            # All Joints and Muscles can be found in the "*.osim"
	            # Obtain the joint data from model "arm_26"
	            # In arm_26, the joint set is [offset, r_shoulder, r_elbow]
	            s_val = self.sim_manager.get_model_property("r_shoulder", datapack_type="Joint")
	            e_val = self.sim_manager.get_model_property("r_elbow", datapack_type="Joint") 
	            # Send data to TF
	            self._setDataPack("joints", {"shoulder": s_val, "elbow": e_val})
	            self._setDataPack("infos", {"time": self.sim_manager.get_sim_time()})
	        # Set muscles' force to change joints
	        self.sim_manager.run_step(self.action)
	        # To show components in the model changed by action
	        # 1: To show components in a list
	        #ctrl_list = self.sim_manager.theWorld.model.getControlsTable()
	        # 2: To show components one by one
	        #print(self.sim_manager.get_model_properties("Force"))
	
	    def reset(self):
	        print("resetting the opensim simulation...")
	        # Reset the value of set datapacks
	        self._setDataPack("joints", {"shoulder": 0, "elbow": 0})
	        self._setDataPack("infos", {"time": 0})
	        # Reset simulation model
	        self.sim_manager.reset()
	        
	    def shutdown(self):
	        print("Engine 1 is shutting down")



.. _doxid-opensim_engine_1opensim_simulator_manager:

SimulatorManager
~~~~~~~~~~~~~~~~

The SimulatorManager is a python class that acts as a bridge to connect NRP-core and OpenSim simulations through the Python API of the latter. It processes requests for simulation initialization, reset, shutdown, run step, and information retrival.

When instantiated, it loads the opensim model specified in the engine configuration. Additionally a controller is attached to the model with actuators for every muscle in the model (as returned by `getMuscles() <https://simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1Model.html#ae11107384d66ad029a572fae379785c8>`__). The control function for each muscle is of type `Constant <https://simtk.org/api_docs/opensim/api_docs20b/classOpenSim_1_1Constant.html>`__, with a default value of ``1.0``.

The following functions are available to interact with the Opensim simulation:

* ``run_step(action)`` : advances the simulation by the engine timestep as specified in the engine configuration. Takes as input an array of floats. The number of its elements must be equal to the number of muscles in the model. They are set as value for each muscle constant control function, in the same order as they are stored in the model controller ``ControlFunctions`` property

* ``reset()`` : resets the simulation (Note: this function is not hooked to the C++ reset function of NRP-core yet)

* ``get_model_properties(p_type)`` : returns a list with the names of elements of the type specified by *p_type*. The latter can take two possible values: "Joint" (returns the names of the elements in the model ``JointSet``), "Force" (returns the name of the elements in the model ``ForceSet``)

* ``get_model_property(p_name, p_type)`` : returns the observed value for ``p_name`` :
  
  * Joint: represents the angle between two body components
  
  * Force: represents a force applied to bodies or generalized coordinates during a simulation

* ``get_sim_time()`` : returns the simulation time in seconds





.. _doxid-opensim_engine_1opensim_json_datapacks:

DataPacks
~~~~~~~~~

Similarly to the Python JSON engine, the OpenSim engine supports a unique datapack type: *JsonDataPack*. Refer to this :ref:`section <doxid-python_json_engine_1python_json_datapacks>` for more details.





.. _doxid-opensim_engine_1engine_opensim_config_section:

Engine Configuration Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The parameters for this engine are defined in the OpenSimEngine schema (listed :ref:`here <doxid-opensim_engine_1engine_opensim_schema>`), which in turn is based on :ref:`EngineBase <doxid-engine_base_schema>` and :ref:`EngineJSON <doxid-engine_comm_1engine_comm_protocols_schema>` schemas, and thus inherits all parameters from them.

To use the Python Simulator engine in an experiment, set ``EngineType`` to **"opensim"**.

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
WorldFileName   Path to the file of simulation world                        string           X                
Visualiser      To show the simulation in visualizer or not                 bool    false                     
==============  ==========================================================  ======  =======  ========  =====





.. _doxid-opensim_engine_1engine_opensim_schema:

Schema
~~~~~~

As explained above, the schema used by the OpenSim engine inherits from :ref:`EngineBase <doxid-engine_base_schema>` and :ref:`EngineJSON <doxid-engine_comm_1engine_comm_protocols_schema>` schemas. A complete schema for the configuration of this engine is given below:

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


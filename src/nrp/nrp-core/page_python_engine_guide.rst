.. index:: pair: page; Using the Python JSON engine
.. _doxid-python_engine_guide:

Using the Python JSON engine
============================

The :ref:`Python JSON engine <doxid-python_json_engine>` can be used to easily integrate simulators with a Python API. There is a minimal working example with two Python JSON engines exchanging simulation time, located in *examples/tf_exchange* directory. We explain how to run it on :ref:`this page <doxid-running_example_exp_1getting_started_experiment_tf_exchange>`.

In order to create your engine, you can use *examples/tf_exchange/engine_1.py* (listed below) as a reference. Just modify the following methods of the ``Script`` class to suit your needs:

* ``initialize()`` : executed when the engine is initialized

* ``runLoop(timestep)`` : executed when the engine is requested to advance its simulation (from EngineClient::runLoopStep)

* ``shutdown()`` : executed when the engine is requested to shutdown

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

More details about the ``EngineScript`` and other Python JSON engine components can be found :ref:`here <doxid-python_json_engine_1python_json_engine_script>`.


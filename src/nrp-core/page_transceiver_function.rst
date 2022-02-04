.. index:: pair: page; Transceiver Functions
.. _doxid-transceiver_function:

Transceiver Functions
=====================

TransceiverFunctions (TFs) are user-defined Python functions that facilitate the exchange of data between Engines. This data is always stored in :ref:`DataPacks <doxid-datapacks>`. TFs are used in the architecture to convert, transform or combine data from one or multiple engines and relay it to another one.

To identify a Python function as a :ref:`TransceiverFunction <doxid-class_transceiver_function>`, it must be marked with the decorator:

.. ref-code-block:: cpp

	@:ref:`TransceiverFunction <doxid-class_transceiver_function>`("engine_name")

This will both allow this function to be registered upon simulation startup as well as link it to the Engine specified by engine_name. For more details, see this :ref:`section <doxid-transceiver_function_1transceiver_function_implementation>`.

To request datapacks from engines, additional decorators can be prepended to the TF, with the form

.. ref-code-block:: cpp

	@:ref:`EngineDataPack <doxid-class_engine_data_pack>`(keyword, id)

This will configure the TF to request a datapack identified by ``id``, and make it available as ``keyword`` in the TF.

Additionally, TFs allow users to send data back to engines as well. This is accomplished via the return value of the TF, which is expected to be an array of datapacks. After the TF returns, each of these datapacks is sent to the engine it is registered with IF AND ONLY IF this engine is being synchronized in the same loop step in which the TF is executed (see :ref:`here <doxid-sync_model_details_1step_structure>` for more details about the synchronization loop structure).



.. _doxid-transceiver_function_1transceiver_function_example:

Example
~~~~~~~

An example of a TF is listed below. The Python function "transceiver_function" is declared as a TF by prepending the :ref:`TransceiverFunction <doxid-class_transceiver_function>` decorator to its definition. The decorator also specifies that the TF is linked to the engine *python_2*. Using the :ref:`EngineDataPack <doxid-class_engine_data_pack>` decorator, it reads :ref:`DataPack <doxid-class_data_pack>` data corresponding to datapack with id *datapack1* from an Engine with name *python_1* and sends its output to the engine it is linked to, ie. *python_2*. Before the TF is executed, *datapack1* is retrieved from *python_1*, a Python object instance of *datapack1* type Python wrapper is created and passed as argument to the TF with name *datapack_python*. During its execution, the TF first creates a datapack *rec_datapack1* linked to *python_2*. Then, it takes the data from *datapack_python* and places it into *rec_datapack1*, which is then returned from the TF. After execution of this TF, *rec_datapack1* will be sent to *python_2*.

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

This TF is part of an example experiment that can be found in the *examples/tf_exchange* folder.





.. _doxid-transceiver_function_1transceiver_function_synchronization:

Synchronization
~~~~~~~~~~~~~~~

To ensure that output datapacks from a TF are *systematically* received by their engines it is **strongly recommended** that they return only datapacks linked to the same engine as the TF itself. The reason for this is because TFs will always be executed in the simulation loop in which their linked engines are return data to, and receive data from NRP Core. In that case, it is guaranteed that output datapacks prepared by the linked TFs will always be sent to them. Returning datapacks linked to other (non-linked) engines is allowed to avoid duplicating potentially costly computations. Nevertheless, it must be understood that whether or not these non-linked engines actually receive these datapacks depends on their own time step. In other terms, when a datapack D for engine A is prepared in a TF linked to engine B, then by design D may not always reach A.





.. _doxid-transceiver_function_1transceiver_function_implementation:

Implementation Details
~~~~~~~~~~~~~~~~~~~~~~

TransceiverFunctions are managed by the :ref:`TransceiverFunctionManager <doxid-class_transceiver_function_manager>` and the :ref:`TransceiverFunctionInterpreter <doxid-class_transceiver_function_interpreter>`. The former deals with general tasks such as loading TFs from the experiment configuration and de-/activation of TFs, while the latter handles the actual Python script execution.

On Simulation Loop configuration stage, all :ref:`TransceiverFunction <doxid-class_transceiver_function>` configurations are read. For each config, the Python script that contains the TF is loaded and executed. Inside of the scripts, TFs must be registered with the :ref:`TransceiverFunctionInterpreter <doxid-class_transceiver_function_interpreter>` by calling :ref:`TransceiverFunctionInterpreter::registerNewTransceiverFunction <doxid-class_transceiver_function_interpreter_1afd18ead4de0062ebf9e7ff2ee0cd697f>`. To automatize this process the :ref:`TransceiverFunction <doxid-class_transceiver_function>` decorator is provided. Any function in the TF script preceded by this decorator will be registered as a TF.

Currently, all TFs share the same :ref:`PythonInterpreterState <doxid-class_python_interpreter_state>`, meaning they share the same global and local Python variable pool. Therefore, a function/variable defined in one TF Python script is accessible in others.


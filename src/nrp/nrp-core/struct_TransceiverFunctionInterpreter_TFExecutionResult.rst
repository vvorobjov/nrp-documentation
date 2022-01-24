.. index:: pair: struct; TransceiverFunctionInterpreter::TFExecutionResult
.. _doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result:

struct TransceiverFunctionInterpreter::TFExecutionResult
========================================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Result of a single TF run. :ref:`More...<details-struct_transceiver_function_interpreter_1_1_t_f_execution_result>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <transceiver_function_interpreter.h>
	
	struct TFExecutionResult {
		// fields
	
		:ref:`datapack_list_t<doxid-class_transceiver_function_interpreter_1ab12e9c929a56cbb8bcce72a3bd63cdbb>` :ref:`DataPackList<doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result_1afe0851a7c8ec4500ffdbcecfc486dba6>`;
		std::vector<:ref:`DataPackInterface<doxid-class_data_pack_interface>`*> :ref:`DataPacks<doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result_1a7199d055066167465862f028a0562f14>`;

		// construction
	
		:target:`TFExecutionResult<doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result_1a88edb305d6adbbadbc576b7b7918e59d>`(:ref:`datapack_list_t<doxid-class_transceiver_function_interpreter_1ab12e9c929a56cbb8bcce72a3bd63cdbb>`&& _datapackList);

		// methods
	
		void :ref:`extractDataPacks<doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result_1a60b3f6bb2942a0ba75542ef1aacaf689>`();
	};
.. _details-struct_transceiver_function_interpreter_1_1_t_f_execution_result:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Result of a single TF run.

Fields
------

.. index:: pair: variable; DataPackList
.. _doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result_1afe0851a7c8ec4500ffdbcecfc486dba6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`datapack_list_t<doxid-class_transceiver_function_interpreter_1ab12e9c929a56cbb8bcce72a3bd63cdbb>` DataPackList

Python :ref:`DataPack <doxid-class_data_pack>` List.

.. index:: pair: variable; DataPacks
.. _doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result_1a7199d055066167465862f028a0562f14:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::vector<:ref:`DataPackInterface<doxid-class_data_pack_interface>`*> DataPacks

Extracted Pointers to datapacks in DataPackList.

Methods
-------

.. index:: pair: function; extractDataPacks
.. _doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result_1a60b3f6bb2942a0ba75542ef1aacaf689:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void extractDataPacks()

Extract datapacks from DataPackList and insert them into DataPacks.


.. index:: pair: struct; TransceiverFunctionData
.. _doxid-struct_transceiver_function_data:

struct TransceiverFunctionData
==============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Data associated with a single transceiver function. :ref:`More...<details-struct_transceiver_function_data>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <transceiver_function_interpreter.h>
	
	struct TransceiverFunctionData {
		// fields
	
		std::string :ref:`Name<doxid-struct_transceiver_function_data_1a2be452ade0b30b493c4fe1e450270a5b>`;
		:ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :ref:`TransceiverFunction<doxid-struct_transceiver_function_data_1a1a5085d49d0052d9f41be649d8efd987>` = nullptr;
		:ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` :ref:`DataPackIDs<doxid-struct_transceiver_function_data_1a10f676d519abad10b4406c6e92284abb>`;
		boost::python::object :ref:`LocalVariables<doxid-struct_transceiver_function_data_1a6b9c0314bff529b4b0a15d131f3c41cd>`;

		// construction
	
		:target:`TransceiverFunctionData<doxid-struct_transceiver_function_data_1a2b2f9b8ea0ae0cd5231770e0cc90fd10>`();
	
		:target:`TransceiverFunctionData<doxid-struct_transceiver_function_data_1a090bc5dcf081b9cf982c88c51220dbb3>`(
			const std::string& _name,
			const :ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`& _transceiverFunction,
			const :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`& _datapackIDs,
			const boost::python::object& _localVariables
		);
	};
.. _details-struct_transceiver_function_data:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Data associated with a single transceiver function.

Fields
------

.. index:: pair: variable; Name
.. _doxid-struct_transceiver_function_data_1a2be452ade0b30b493c4fe1e450270a5b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::string Name

Name of Transfer Function.

.. index:: pair: variable; TransceiverFunction
.. _doxid-struct_transceiver_function_data_1a1a5085d49d0052d9f41be649d8efd987:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` TransceiverFunction = nullptr

Pointer to :ref:`TransceiverFunction <doxid-class_transceiver_function>`.

.. index:: pair: variable; DataPackIDs
.. _doxid-struct_transceiver_function_data_1a10f676d519abad10b4406c6e92284abb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` DataPackIDs

DataPacks requested by TF.

.. index:: pair: variable; LocalVariables
.. _doxid-struct_transceiver_function_data_1a6b9c0314bff529b4b0a15d131f3c41cd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object LocalVariables

Local variables used by this :ref:`TransceiverFunction <doxid-class_transceiver_function>`.


.. index:: pair: class; TransceiverFunctionInterpreter
.. _doxid-class_transceiver_function_interpreter:

class TransceiverFunctionInterpreter
====================================

.. toctree::
	:hidden:

	struct_TransceiverFunctionInterpreter_TFExecutionResult.rst

Overview
~~~~~~~~

Python Interpreter to manage transfer function calls. :ref:`More...<details-class_transceiver_function_interpreter>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <transceiver_function_interpreter.h>
	
	class TransceiverFunctionInterpreter {
	public:
		// typedefs
	
		typedef boost::python::list :target:`datapack_list_t<doxid-class_transceiver_function_interpreter_1ab12e9c929a56cbb8bcce72a3bd63cdbb>`;
		typedef std::map<std::string, const :ref:`EngineClientInterface::datapacks_t<doxid-class_engine_client_interface_1a2562bba1cd7ecc32dc6bfd54691c669c>`*> :target:`engines_datapacks_t<doxid-class_transceiver_function_interpreter_1a9ceee8bdb2310bc618cd9916814c1626>`;
		typedef std::multimap<std::string, :ref:`TransceiverFunctionData<doxid-struct_transceiver_function_data>`> :target:`transceiver_function_datas_t<doxid-class_transceiver_function_interpreter_1a59f70f2acb7ce7d197d80b4bd44ada08>`;
		typedef std::pair<TransceiverFunctionInterpreter::transceiver_function_datas_t::iterator, TransceiverFunctionInterpreter::transceiver_function_datas_t::iterator> :target:`linked_tfs_t<doxid-class_transceiver_function_interpreter_1a2623b29fe3877f8bd03daccd375d931d>`;

		// structs
	
		struct :ref:`TFExecutionResult<doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result>`;

		// construction
	
		:target:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter_1a1be1e76736ce6b167bc53680ebfc5f7d>`();
		:target:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter_1ae808c9e0c482c4dc046a66c536254705>`(const boost::python::dict& tfGlobals);

		// methods
	
		transceiver_function_datas_t::const_iterator :ref:`findTransceiverFunction<doxid-class_transceiver_function_interpreter_1a0e08139fd162d03406e12f58ae5b638e>`(const std::string& name) const;
		const :ref:`transceiver_function_datas_t<doxid-class_transceiver_function_interpreter_1a59f70f2acb7ce7d197d80b4bd44ada08>`& :ref:`getLoadedTransceiverFunctions<doxid-class_transceiver_function_interpreter_1a8b3c8dc5b53fb6580888626c8bdfe3dd>`() const;
		:ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` :ref:`updateRequestedDataPackIDs<doxid-class_transceiver_function_interpreter_1a2e60bf7c6124b0ef8598e366c54deec1>`() const;
		void :ref:`setEngineDataPacks<doxid-class_transceiver_function_interpreter_1ab585bc27bd7c6a678fc38dd2367ac6ac>`(:ref:`engines_datapacks_t<doxid-class_transceiver_function_interpreter_1a9ceee8bdb2310bc618cd9916814c1626>`&& engineDataPacks);
		constexpr const :ref:`engines_datapacks_t<doxid-class_transceiver_function_interpreter_1a9ceee8bdb2310bc618cd9916814c1626>`& :ref:`getEngineDataPacks<doxid-class_transceiver_function_interpreter_1abc998064710edd93d6ef2841db3e928e>`() const;
		boost::python::object :ref:`runSingleTransceiverFunction<doxid-class_transceiver_function_interpreter_1a4cb9a77e315d52a544b64c4b9ae5dac1>`(const std::string& tfName);
		boost::python::object :ref:`runSingleTransceiverFunction<doxid-class_transceiver_function_interpreter_1a28ebbb2f5732fc161e78d89bf52251ae>`(const :ref:`TransceiverFunctionData<doxid-struct_transceiver_function_data>`& tfData);
		:ref:`linked_tfs_t<doxid-class_transceiver_function_interpreter_1a2623b29fe3877f8bd03daccd375d931d>` :ref:`getLinkedTransceiverFunctions<doxid-class_transceiver_function_interpreter_1a50faa93ad5bdde7c2efb59a5b68a2ed3>`(const std::string& engineName);
		transceiver_function_datas_t::iterator :ref:`loadTransceiverFunction<doxid-class_transceiver_function_interpreter_1ab03b0ff70ec079da12413e8cc3f11a4a>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& transceiverFunction);
	
		transceiver_function_datas_t::iterator :ref:`loadTransceiverFunction<doxid-class_transceiver_function_interpreter_1aa1f7fe4a9e97fcbc3a1768a40061e07d>`(
			const std::string& tfName,
			const :ref:`TransceiverDataPackInterfaceSharedPtr<doxid-transceiver__datapack__interface_8h_1a32d7478b1d3bfae5d84644961d494d1a>`& transceiverFunction,
			boost::python::object&& localVars = boost::python::object()
		);
	
		transceiver_function_datas_t::iterator :ref:`updateTransceiverFunction<doxid-class_transceiver_function_interpreter_1a7c3d6d561b2b8e5809d6e6d73c85190b>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& transceiverFunction);
	};
.. _details-class_transceiver_function_interpreter:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Python Interpreter to manage transfer function calls.

Methods
-------

.. index:: pair: function; findTransceiverFunction
.. _doxid-class_transceiver_function_interpreter_1a0e08139fd162d03406e12f58ae5b638e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	transceiver_function_datas_t::const_iterator findTransceiverFunction(const std::string& name) const

Find TF with given name.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- name

		- Name to find



.. rubric:: Returns:

Returns iterator to TF. If name not present, returns _transceiverFunctions.end()

.. index:: pair: function; getLoadedTransceiverFunctions
.. _doxid-class_transceiver_function_interpreter_1a8b3c8dc5b53fb6580888626c8bdfe3dd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const :ref:`transceiver_function_datas_t<doxid-class_transceiver_function_interpreter_1a59f70f2acb7ce7d197d80b4bd44ada08>`& getLoadedTransceiverFunctions() const

Reference to loaded TFs.



.. rubric:: Returns:

Returns reference to loaded TFs

.. index:: pair: function; updateRequestedDataPackIDs
.. _doxid-class_transceiver_function_interpreter_1a2e60bf7c6124b0ef8598e366c54deec1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` updateRequestedDataPackIDs() const

Get :ref:`DataPack <doxid-class_data_pack>` IDs requested by TFs.



.. rubric:: Returns:

.. index:: pair: function; setEngineDataPacks
.. _doxid-class_transceiver_function_interpreter_1ab585bc27bd7c6a678fc38dd2367ac6ac:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void setEngineDataPacks(:ref:`engines_datapacks_t<doxid-class_transceiver_function_interpreter_1a9ceee8bdb2310bc618cd9916814c1626>`&& engineDataPacks)

Set :ref:`EngineClientInterface <doxid-class_engine_client_interface>` pointers. Used by TransceiverFunctions to access datapacks.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engines

		- Mapping from engine name to engine ptr

.. index:: pair: function; getEngineDataPacks
.. _doxid-class_transceiver_function_interpreter_1abc998064710edd93d6ef2841db3e928e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	constexpr const :ref:`engines_datapacks_t<doxid-class_transceiver_function_interpreter_1a9ceee8bdb2310bc618cd9916814c1626>`& getEngineDataPacks() const

Access engine map.

.. index:: pair: function; runSingleTransceiverFunction
.. _doxid-class_transceiver_function_interpreter_1a4cb9a77e315d52a544b64c4b9ae5dac1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object runSingleTransceiverFunction(const std::string& tfName)

Execute one transfer function.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- tfName

		- Name of function to execute



.. rubric:: Returns:

Returns result of execution. Contains a list of datapack commands

.. index:: pair: function; runSingleTransceiverFunction
.. _doxid-class_transceiver_function_interpreter_1a28ebbb2f5732fc161e78d89bf52251ae:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object runSingleTransceiverFunction(const :ref:`TransceiverFunctionData<doxid-struct_transceiver_function_data>`& tfData)

Execute one transfer function.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- tfData

		- :ref:`TransceiverFunction <doxid-class_transceiver_function>` to execute



.. rubric:: Returns:

Returns result of execution. Contains a list of datapack commands

.. index:: pair: function; getLinkedTransceiverFunctions
.. _doxid-class_transceiver_function_interpreter_1a50faa93ad5bdde7c2efb59a5b68a2ed3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`linked_tfs_t<doxid-class_transceiver_function_interpreter_1a2623b29fe3877f8bd03daccd375d931d>` getLinkedTransceiverFunctions(const std::string& engineName)

Get TFs linked to specific engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineName

		- Name of engine



.. rubric:: Returns:

Returns range of TFs linked to engine name

.. index:: pair: function; loadTransceiverFunction
.. _doxid-class_transceiver_function_interpreter_1ab03b0ff70ec079da12413e8cc3f11a4a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	transceiver_function_datas_t::iterator loadTransceiverFunction(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& transceiverFunction)

Prepares a TF for execution. Loads code into storage.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- transceiverFunction

		- Pointer to TF configuration



.. rubric:: Returns:

Returns iterator to loaded TF

.. index:: pair: function; loadTransceiverFunction
.. _doxid-class_transceiver_function_interpreter_1aa1f7fe4a9e97fcbc3a1768a40061e07d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	transceiver_function_datas_t::iterator loadTransceiverFunction(
		const std::string& tfName,
		const :ref:`TransceiverDataPackInterfaceSharedPtr<doxid-transceiver__datapack__interface_8h_1a32d7478b1d3bfae5d84644961d494d1a>`& transceiverFunction,
		boost::python::object&& localVars = boost::python::object()
	)

Prepares a TF for execution. Loads code into storage.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- tfName

		- Name of this TF

	*
		- transceiverFunction

		- Ptr to the TF

	*
		- localVars

		- Local Python variables required by this TF



.. rubric:: Returns:

Returns iterator to loaded TF

.. index:: pair: function; updateTransceiverFunction
.. _doxid-class_transceiver_function_interpreter_1a7c3d6d561b2b8e5809d6e6d73c85190b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	transceiver_function_datas_t::iterator updateTransceiverFunction(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& transceiverFunction)

Updates TF with the given name.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- name

		- Name of TF

	*
		- transceiverFunction

		- New TF configuration



.. rubric:: Returns:

Returns iterator to updated TF


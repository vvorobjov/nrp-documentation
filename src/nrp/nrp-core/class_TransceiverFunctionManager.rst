.. index:: pair: class; TransceiverFunctionManager
.. _doxid-class_transceiver_function_manager:

class TransceiverFunctionManager
================================

.. toctree::
	:hidden:

	struct_TransceiverFunctionManager_less_tf_settings.rst

Overview
~~~~~~~~

Manages all available/active transfer functions. :ref:`More...<details-class_transceiver_function_manager>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <transceiver_function_manager.h>
	
	class TransceiverFunctionManager {
	public:
		// typedefs
	
		typedef std::set<:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`, less_tf_settings> :target:`tf_settings_t<doxid-class_transceiver_function_manager_1a2e0c07cda83bb5110cb795c343c1ebb7>`;
		typedef std::list<:ref:`TransceiverFunctionInterpreter::TFExecutionResult<doxid-struct_transceiver_function_interpreter_1_1_t_f_execution_result>`> :target:`tf_results_t<doxid-class_transceiver_function_manager_1a896a151a16f978eb6ef7d7f2e92f5ee2>`;

		// structs
	
		struct :ref:`less_tf_settings<doxid-struct_transceiver_function_manager_1_1less__tf__settings>`;

		// construction
	
		:target:`TransceiverFunctionManager<doxid-class_transceiver_function_manager_1af696c071d99cf6b5744d9a63652e9b20>`();
		:target:`TransceiverFunctionManager<doxid-class_transceiver_function_manager_1a93695946e2e3e9523ea5e17eb943a14c>`(boost::python::dict tfGlobals);

		// methods
	
		:ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` :ref:`updateRequestedDataPackIDs<doxid-class_transceiver_function_manager_1a1d4270468fac341a5ead8f7346e9827d>`() const;
		void :ref:`loadTF<doxid-class_transceiver_function_manager_1a25051b4f7115fd13c539c356b7e76695>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& tfConfig);
		void :ref:`updateTF<doxid-class_transceiver_function_manager_1aea4f1edf8ca43e8133035ed1ad9f95c2>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& tfConfig);
		:ref:`tf_results_t<doxid-class_transceiver_function_manager_1a896a151a16f978eb6ef7d7f2e92f5ee2>` :ref:`executeActiveLinkedPFs<doxid-class_transceiver_function_manager_1a6f8bc7883b64727a273f7d3793b3a0b9>`(const std::string& engineName);
		:ref:`tf_results_t<doxid-class_transceiver_function_manager_1a896a151a16f978eb6ef7d7f2e92f5ee2>` :ref:`executeActiveLinkedTFs<doxid-class_transceiver_function_manager_1a88f39bcab252d20707819fb5d76d5d7e>`(const std::string& engineName);
		:ref:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter>`& :ref:`getInterpreter<doxid-class_transceiver_function_manager_1a6f569a5a02547c6ee5ee04c2f7cf06a0>`();
	};
.. _details-class_transceiver_function_manager:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Manages all available/active transfer functions.

Methods
-------

.. index:: pair: function; updateRequestedDataPackIDs
.. _doxid-class_transceiver_function_manager_1a1d4270468fac341a5ead8f7346e9827d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` updateRequestedDataPackIDs() const

Return list of datapacks that the TFs request.



.. rubric:: Returns:

Returns container with all requested datapack IDs

.. index:: pair: function; loadTF
.. _doxid-class_transceiver_function_manager_1a25051b4f7115fd13c539c356b7e76695:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void loadTF(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& tfConfig)

Load TF from given configuration.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- tfConfig

		- TF Configuration

	*
		- Throws

		- an exception if a TF with the same name is already loaded. Use updateTF to change loaded TFs

.. index:: pair: function; updateTF
.. _doxid-class_transceiver_function_manager_1aea4f1edf8ca43e8133035ed1ad9f95c2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void updateTF(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& tfConfig)

Updates an existing TF or creates a new one.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- Name

		- of old TF

	*
		- tfConfig

		- TF Configuration

.. index:: pair: function; executeActiveLinkedPFs
.. _doxid-class_transceiver_function_manager_1a6f8bc7883b64727a273f7d3793b3a0b9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`tf_results_t<doxid-class_transceiver_function_manager_1a896a151a16f978eb6ef7d7f2e92f5ee2>` executeActiveLinkedPFs(const std::string& engineName)

Execute all preprocessing TFs linked to an engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineName

		- Name of engine



.. rubric:: Returns:

Returns results of linked preprocessing TFs

.. index:: pair: function; executeActiveLinkedTFs
.. _doxid-class_transceiver_function_manager_1a88f39bcab252d20707819fb5d76d5d7e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`tf_results_t<doxid-class_transceiver_function_manager_1a896a151a16f978eb6ef7d7f2e92f5ee2>` executeActiveLinkedTFs(const std::string& engineName)

Execute all TFs linked to an engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineName

		- Name of engine



.. rubric:: Returns:

Returns results of linked TFs

.. index:: pair: function; getInterpreter
.. _doxid-class_transceiver_function_manager_1a6f569a5a02547c6ee5ee04c2f7cf06a0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter>`& getInterpreter()

Get TF Interpreter.


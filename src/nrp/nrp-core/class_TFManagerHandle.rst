.. index:: pair: class; TFManagerHandle
.. _doxid-class_t_f_manager_handle:

class TFManagerHandle
=====================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Uses the TF framework to execute datapack transformation operations. :ref:`More...<details-class_t_f_manager_handle>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <tf_manager_handle.h>
	
	class TFManagerHandle: public :ref:`DataPackProcessor<doxid-class_data_pack_processor>` {
	public:
		// methods
	
		virtual void :ref:`init<doxid-class_t_f_manager_handle_1a2443878d8b6c10c01c6198fa370a28f6>`(
			const :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& simConfig,
			const :ref:`engine_interfaces_t<doxid-class_data_pack_processor_1a269b18f42f5acedb594ad2912c795824>`& engines
		);
	
		virtual void :ref:`updateDataPacksFromEngines<doxid-class_t_f_manager_handle_1a74342bbae9037dd9284f631fcafe26cb>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines);
		virtual void :ref:`compute<doxid-class_t_f_manager_handle_1afffb376ad5e96c8cac127a52e13182aa>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines);
		virtual void :ref:`sendDataPacksToEngines<doxid-class_t_f_manager_handle_1a6e9bc567ab328f0071445d3baf387cbc>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines);
	
		static void :ref:`executePreprocessingFunctions<doxid-class_t_f_manager_handle_1a1efba5d6971dfa2a7fa6e0e448dc053a>`(
			:ref:`TransceiverFunctionManager<doxid-class_transceiver_function_manager>`& tfManager,
			const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines
		);
	
		static :ref:`TransceiverFunctionSortedResults<doxid-struct_transceiver_function_sorted_results>` :ref:`executeTransceiverFunctions<doxid-class_t_f_manager_handle_1a0824faabbf7977f8d5a5c0384efa6714>`(
			:ref:`TransceiverFunctionManager<doxid-class_transceiver_function_manager>`& tfManager,
			const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines
		);
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// typedefs
	
		typedef std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`> :ref:`engine_interfaces_t<doxid-class_data_pack_processor_1a269b18f42f5acedb594ad2912c795824>`;

		// methods
	
		virtual void :ref:`init<doxid-class_data_pack_processor_1a85eace47761c625f2526e6f0efb51c83>`(
			const :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& simConfig,
			const :ref:`engine_interfaces_t<doxid-class_data_pack_processor_1a269b18f42f5acedb594ad2912c795824>`& engines
		) = 0;
	
		virtual void :ref:`updateDataPacksFromEngines<doxid-class_data_pack_processor_1a24a4cf8e245dd8526f3b451dcb78cefd>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines) = 0;
		virtual void :ref:`compute<doxid-class_data_pack_processor_1a89859a99d685bf7a495b8c3a1e644717>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines) = 0;
		virtual void :ref:`sendDataPacksToEngines<doxid-class_data_pack_processor_1ad980b1d73a944d8b3c6faf3777b9b488>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines) = 0;
		void :ref:`datapackCycle<doxid-class_data_pack_processor_1a63fa2e4c1c411be5a2b2c58ad8507c4e>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines);

.. _details-class_t_f_manager_handle:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Uses the TF framework to execute datapack transformation operations.

Methods
-------

.. index:: pair: function; init
.. _doxid-class_t_f_manager_handle_1a2443878d8b6c10c01c6198fa370a28f6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void init(
		const :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& simConfig,
		const :ref:`engine_interfaces_t<doxid-class_data_pack_processor_1a269b18f42f5acedb594ad2912c795824>`& engines
	)

Initializes the handler.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- simConfig

		- json object containing configuration information to initialize the handler

	*
		- engines

		- list of Engine clients participating in the simulation

.. index:: pair: function; updateDataPacksFromEngines
.. _doxid-class_t_f_manager_handle_1a74342bbae9037dd9284f631fcafe26cb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void updateDataPacksFromEngines(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines)

Request datapacks from engines.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engines

		- Engines that are been synchronize in the current loop

.. index:: pair: function; compute
.. _doxid-class_t_f_manager_handle_1afffb376ad5e96c8cac127a52e13182aa:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void compute(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines)

Perform computations on datapacks.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engines

		- Engines that are been synchronize in the current loop

.. index:: pair: function; sendDataPacksToEngines
.. _doxid-class_t_f_manager_handle_1a6e9bc567ab328f0071445d3baf387cbc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void sendDataPacksToEngines(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines)

Send datapacks to engines.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engines

		- Engines that are been synchronize in the current loop

.. index:: pair: function; executePreprocessingFunctions
.. _doxid-class_t_f_manager_handle_1a1efba5d6971dfa2a7fa6e0e448dc053a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static void executePreprocessingFunctions(
		:ref:`TransceiverFunctionManager<doxid-class_transceiver_function_manager>`& tfManager,
		const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines
	)

Execute PreprocessingFunctions for each engine and place output datapacks in its cache.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- tfManager

		- tfManager

	*
		- engines

		- Engines that are been synchronize in the current loop

.. index:: pair: function; executeTransceiverFunctions
.. _doxid-class_t_f_manager_handle_1a0824faabbf7977f8d5a5c0384efa6714:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static :ref:`TransceiverFunctionSortedResults<doxid-struct_transceiver_function_sorted_results>` executeTransceiverFunctions(
		:ref:`TransceiverFunctionManager<doxid-class_transceiver_function_manager>`& tfManager,
		const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines
	)

Execute TransceiverFunctions for each engine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- tfManager

		- tfManager

	*
		- engines

		- Engines that are been synchronize in the current loop


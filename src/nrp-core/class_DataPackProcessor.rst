.. index:: pair: class; DataPackProcessor
.. _doxid-class_data_pack_processor:

class DataPackProcessor
=======================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Helper class for :ref:`FTILoop <doxid-class_f_t_i_loop>` encapsulating the datapack operations between Engines in a simulation loop. :ref:`More...<details-class_data_pack_processor>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <datapack_handle.h>
	
	class DataPackProcessor {
	public:
		// typedefs
	
		typedef std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`> :target:`engine_interfaces_t<doxid-class_data_pack_processor_1a269b18f42f5acedb594ad2912c795824>`;

		// methods
	
		virtual void :ref:`init<doxid-class_data_pack_processor_1a85eace47761c625f2526e6f0efb51c83>`(
			const :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& simConfig,
			const :ref:`engine_interfaces_t<doxid-class_data_pack_processor_1a269b18f42f5acedb594ad2912c795824>`& engines
		) = 0;
	
		virtual void :ref:`updateDataPacksFromEngines<doxid-class_data_pack_processor_1a24a4cf8e245dd8526f3b451dcb78cefd>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines) = 0;
		virtual void :ref:`compute<doxid-class_data_pack_processor_1a89859a99d685bf7a495b8c3a1e644717>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines) = 0;
		virtual void :ref:`sendDataPacksToEngines<doxid-class_data_pack_processor_1ad980b1d73a944d8b3c6faf3777b9b488>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines) = 0;
		void :ref:`datapackCycle<doxid-class_data_pack_processor_1a63fa2e4c1c411be5a2b2c58ad8507c4e>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines);
	};

	// direct descendants

	struct :ref:`ComputationalGraphHandle<doxid-struct_computational_graph_handle>`;
	class :ref:`TFManagerHandle<doxid-class_t_f_manager_handle>`;
.. _details-class_data_pack_processor:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Helper class for :ref:`FTILoop <doxid-class_f_t_i_loop>` encapsulating the datapack operations between Engines in a simulation loop.

Methods
-------

.. index:: pair: function; init
.. _doxid-class_data_pack_processor_1a85eace47761c625f2526e6f0efb51c83:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void init(
		const :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& simConfig,
		const :ref:`engine_interfaces_t<doxid-class_data_pack_processor_1a269b18f42f5acedb594ad2912c795824>`& engines
	) = 0

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
.. _doxid-class_data_pack_processor_1a24a4cf8e245dd8526f3b451dcb78cefd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void updateDataPacksFromEngines(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines) = 0

Request datapacks from engines.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engines

		- Engines that are been synchronize in the current loop

.. index:: pair: function; compute
.. _doxid-class_data_pack_processor_1a89859a99d685bf7a495b8c3a1e644717:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void compute(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines) = 0

Perform computations on datapacks.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engines

		- Engines that are been synchronize in the current loop

.. index:: pair: function; sendDataPacksToEngines
.. _doxid-class_data_pack_processor_1ad980b1d73a944d8b3c6faf3777b9b488:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void sendDataPacksToEngines(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines) = 0

Send datapacks to engines.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engines

		- Engines that are been synchronize in the current loop

.. index:: pair: function; datapackCycle
.. _doxid-class_data_pack_processor_1a63fa2e4c1c411be5a2b2c58ad8507c4e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void datapackCycle(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines)

Execute sequentially the update, compute and send operations.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engines

		- Engines that are been synchronize in the current loop


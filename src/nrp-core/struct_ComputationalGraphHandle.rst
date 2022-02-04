.. index:: pair: struct; ComputationalGraphHandle
.. _doxid-struct_computational_graph_handle:

struct ComputationalGraphHandle
===============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Uses a Computation Graph to execute datapack transformation operations. :ref:`More...<details-struct_computational_graph_handle>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <computational_graph_handle.h>
	
	struct ComputationalGraphHandle: public :ref:`DataPackProcessor<doxid-class_data_pack_processor>` {
		// fields
	
		bool :target:`_slaveMode<doxid-struct_computational_graph_handle_1ac3c9cd649fea5eef6b9932fc3dbd0a42>`;
		bool :target:`_spinROS<doxid-struct_computational_graph_handle_1ac3c55ba9f686df27e43e4c7261beb854>`;
		PyGILState_STATE :target:`_pyGILState<doxid-struct_computational_graph_handle_1a71c5d9b3d1801a6b3d4d0490438b9c51>`;
		std::map<std::string, :ref:`InputEngineNode<doxid-class_input_engine_node>`*> :ref:`_inputs<doxid-struct_computational_graph_handle_1ab1a8d52ffff96d877b9dffe03aae5afd>`;
		std::map<std::string, :ref:`OutputEngineNode<doxid-class_output_engine_node>`*> :ref:`_outputs<doxid-struct_computational_graph_handle_1ac0c67880e1c1deeb05b110b208613fee>`;

		// construction
	
		:target:`ComputationalGraphHandle<doxid-struct_computational_graph_handle_1a4607d410000106bb5ef13493fb10656d>`(bool slaveMode = false, bool spinROS = false);

		// methods
	
		virtual void :ref:`init<doxid-struct_computational_graph_handle_1ad4795e50c39319f98ed0cf8705a0ce7c>`(
			const :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& simConfig,
			const :ref:`engine_interfaces_t<doxid-class_data_pack_processor_1a269b18f42f5acedb594ad2912c795824>`& engines
		);
	
		virtual void :ref:`updateDataPacksFromEngines<doxid-struct_computational_graph_handle_1ae1bac51ddb9e94aa2ed084e46ed59c34>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines);
		virtual void :ref:`compute<doxid-struct_computational_graph_handle_1a5b3671667f8fe7b88210956df778ebb2>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines);
		virtual void :ref:`sendDataPacksToEngines<doxid-struct_computational_graph_handle_1a0d4e48152a7cd92114aa82a901441520>`(const std::vector<:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`>& engines);
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

.. _details-struct_computational_graph_handle:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Uses a Computation Graph to execute datapack transformation operations.

Fields
------

.. index:: pair: variable; _inputs
.. _doxid-struct_computational_graph_handle_1ab1a8d52ffff96d877b9dffe03aae5afd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::map<std::string, :ref:`InputEngineNode<doxid-class_input_engine_node>`*> _inputs

Map containing all InputEngineNodes associated with this simulation.

.. index:: pair: variable; _outputs
.. _doxid-struct_computational_graph_handle_1ac0c67880e1c1deeb05b110b208613fee:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::map<std::string, :ref:`OutputEngineNode<doxid-class_output_engine_node>`*> _outputs

Map containing all OutputEngineNodes associated with this simulation.

Methods
-------

.. index:: pair: function; init
.. _doxid-struct_computational_graph_handle_1ad4795e50c39319f98ed0cf8705a0ce7c:

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
.. _doxid-struct_computational_graph_handle_1ae1bac51ddb9e94aa2ed084e46ed59c34:

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
.. _doxid-struct_computational_graph_handle_1a5b3671667f8fe7b88210956df778ebb2:

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
.. _doxid-struct_computational_graph_handle_1a0d4e48152a7cd92114aa82a901441520:

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


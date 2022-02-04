.. index:: pair: class; InputNode
.. _doxid-class_input_node:

template class InputNode
========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Implementation of an input node in the computation graph. :ref:`More...<details-class_input_node>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <input_node.h>
	
	template <class DATA>
	class InputNode: public :ref:`ComputationalNode<doxid-class_computational_node>` {
	public:
		// construction
	
		:ref:`InputNode<doxid-class_input_node_1aa1077db83a3e94cfdee0483910a1fb62>`(
			const std::string& id,
			:ref:`InputNodePolicies::MsgPublishPolicy<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>` msgPublishPolicy = InputNodePolicies::MsgPublishPolicy::LAST,
			:ref:`InputNodePolicies::MsgCachePolicy<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>` msgCachePolicy = InputNodePolicies::MsgCachePolicy::KEEP_CACHE,
			size_t queue_size = 10
		);

		// methods
	
		virtual void :ref:`compute<doxid-class_input_node_1ab7d08881d8a20ed03a799be01dde46b5>`();
		void :ref:`registerOutput<doxid-class_input_node_1af15ec049b725a7e02f09650f81bfd97c>`(const std::string& id);
		:ref:`OutputPort<doxid-class_output_port>`<DATA>* :ref:`getSinglePort<doxid-class_input_node_1ad9a253c855c20d797820824b319d4a31>`(const std::string& id);
		:ref:`OutputPort<doxid-class_output_port>`<std::vector<const DATA*>>* :ref:`getListPort<doxid-class_input_node_1af2c401b83ae12585082add95df9d1270>`(const std::string& id);
		:ref:`InputNodePolicies::MsgPublishPolicy<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>` :target:`msgPublishPolicy<doxid-class_input_node_1a4377e88c802c215e7e1174feca8e4735>`();
		:ref:`InputNodePolicies::MsgCachePolicy<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>` :target:`msgCachePolicy<doxid-class_input_node_1ac7c8589e78d9b194e222edfab912bd66>`();
		void :target:`setMsgPublishPolicy<doxid-class_input_node_1a6b9e5b9eaa7a8f1b275b698adacc765c>`(:ref:`InputNodePolicies::MsgPublishPolicy<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>` msgPublishPolicy);
		void :target:`setMsgCachePolicy<doxid-class_input_node_1a7e2197a005d86ee5191598f6e13b45a5>`(:ref:`InputNodePolicies::MsgCachePolicy<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>` msgCachePolicy);
	};

	// direct descendants

	class :ref:`InputDummy<doxid-class_input_dummy>`;
	class :ref:`InputEngineNode<doxid-class_input_engine_node>`;

	template <class MSG_TYPE>
	class :ref:`InputROSNode<doxid-class_input_r_o_s_node>`;

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// enums
	
		enum :ref:`NodeType<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>`;

		// methods
	
		bool :ref:`isVisited<doxid-class_computational_node_1aa657870e632df5c4ed11470d10e65d5f>`() const;
		void :ref:`setVisited<doxid-class_computational_node_1a4d75c538b8bbc2f9c26278d27ad8a434>`(bool visited);
		const std::string& :ref:`id<doxid-class_computational_node_1aeab0953471cf02647c0264c8b474afb5>`() const;
		:ref:`NodeType<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>` :ref:`type<doxid-class_computational_node_1a4cb10cde56ec02dd5e31c5b5498388ed>`() const;
		virtual void :ref:`configure<doxid-class_computational_node_1abf931072caba0154df7a388c515b43d8>`() = 0;
		virtual void :ref:`compute<doxid-class_computational_node_1a593ccb6e371475e628e11253b562a7a2>`() = 0;

.. _details-class_input_node:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Implementation of an input node in the computation graph.

Input nodes are the connection points to feed data into the computation graph from outside. It owns output ports which are used to forward incoming data to other nodes. One node can handle multiple ports. The class is templated with the data type the node can handle. Each node implementation can handle only one data type.

Derived classes of :ref:`InputNode <doxid-class_input_node>` remains responsible for the ownership of the data passed to :ref:`InputNode <doxid-class_input_node>` through the :ref:`updatePortData() <doxid-class_input_node_1a75f17a1fb78db1eead81e35f915fc069>` virtual method.

Construction
------------

.. index:: pair: function; InputNode
.. _doxid-class_input_node_1aa1077db83a3e94cfdee0483910a1fb62:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	InputNode(
		const std::string& id,
		:ref:`InputNodePolicies::MsgPublishPolicy<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>` msgPublishPolicy = InputNodePolicies::MsgPublishPolicy::LAST,
		:ref:`InputNodePolicies::MsgCachePolicy<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>` msgCachePolicy = InputNodePolicies::MsgCachePolicy::KEEP_CACHE,
		size_t queue_size = 10
	)

Constructor.

Methods
-------

.. index:: pair: function; compute
.. _doxid-class_input_node_1ab7d08881d8a20ed03a799be01dde46b5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void compute()

Compute. Updates and sends stored msgs.

.. index:: pair: function; registerOutput
.. _doxid-class_input_node_1af15ec049b725a7e02f09650f81bfd97c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerOutput(const std::string& id)

Registers an Output port with id 'id' with this node.

.. index:: pair: function; getSinglePort
.. _doxid-class_input_node_1ad9a253c855c20d797820824b319d4a31:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`OutputPort<doxid-class_output_port>`<DATA>* getSinglePort(const std::string& id)

Returns a pointer to single output port if the port is registered, nullptr otherwise.

.. index:: pair: function; getListPort
.. _doxid-class_input_node_1af2c401b83ae12585082add95df9d1270:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`OutputPort<doxid-class_output_port>`<std::vector<const DATA*>>* getListPort(const std::string& id)

Returns a pointer to list output port if the port is registered, nullptr otherwise.


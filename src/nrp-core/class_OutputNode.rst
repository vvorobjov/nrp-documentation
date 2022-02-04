.. index:: pair: class; OutputNode
.. _doxid-class_output_node:

template class OutputNode
=========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Implementation of an output node in the computation graph. :ref:`More...<details-class_output_node>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <output_node.h>
	
	template <class DATA>
	class OutputNode: public :ref:`ComputationalNode<doxid-class_computational_node>` {
	public:
		// construction
	
		:ref:`OutputNode<doxid-class_output_node_1a1e4b4fe0f7233e810d08245274f521eb>`(
			const std::string& id,
			:ref:`OutputNodePolicies::MsgPublishPolicy<doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2>` msgPublishPolicy = OutputNodePolicies::MsgPublishPolicy::SERIES,
			int maxPortConnections = 0
		);

		// methods
	
		virtual void :ref:`configure<doxid-class_output_node_1ab38541a1582f99a453254c5ebec2449e>`();
		virtual void :ref:`compute<doxid-class_output_node_1a8bc64d2d817505fbfd68cc266ec7a9f0>`();
	
		template <class T_IN>
		:ref:`InputPort<doxid-class_input_port>`<T_IN, DATA>* :ref:`getOrRegisterInput<doxid-class_output_node_1a833d82862c1d613aab1448c3e0157237>`(const std::string& id);
	
		:ref:`OutputNodePolicies::MsgPublishPolicy<doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2>` :target:`msgPublishPolicy<doxid-class_output_node_1a76f8b117583989b59d210f3c4759d765>`();
	};

	// direct descendants

	class :ref:`OutputDummy<doxid-class_output_dummy>`;
	class :ref:`OutputEngineNode<doxid-class_output_engine_node>`;

	template <class MSG_TYPE>
	class :ref:`OutputROSNode<doxid-class_output_r_o_s_node>`;

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

.. _details-class_output_node:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Implementation of an output node in the computation graph.

Output nodes are the connection point to send data out of the computation graph. In its 'compute' operation all messages received since last 'compute' are sent using the 'sendSingleMsg' implemented by derived classes.

Construction
------------

.. index:: pair: function; OutputNode
.. _doxid-class_output_node_1a1e4b4fe0f7233e810d08245274f521eb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	OutputNode(
		const std::string& id,
		:ref:`OutputNodePolicies::MsgPublishPolicy<doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2>` msgPublishPolicy = OutputNodePolicies::MsgPublishPolicy::SERIES,
		int maxPortConnections = 0
	)

Constructor.

Methods
-------

.. index:: pair: function; configure
.. _doxid-class_output_node_1ab38541a1582f99a453254c5ebec2449e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void configure()

Configures the node making it ready to execute 'compute'.

.. index:: pair: function; compute
.. _doxid-class_output_node_1a8bc64d2d817505fbfd68cc266ec7a9f0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void compute()

Requests the node to execute its computation.

.. index:: pair: function; getOrRegisterInput
.. _doxid-class_output_node_1a833d82862c1d613aab1448c3e0157237:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class T_IN>
	:ref:`InputPort<doxid-class_input_port>`<T_IN, DATA>* getOrRegisterInput(const std::string& id)

Gets or register input port to this node and returns a pointer to it.


.. index:: pair: class; InputROSNode
.. _doxid-class_input_r_o_s_node:

template class InputROSNode
===========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Input node used to connect a ROS subscriber to the computational graph. :ref:`More...<details-class_input_r_o_s_node>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <input_node.h>
	
	template <class MSG_TYPE>
	class InputROSNode: public :ref:`InputNode<doxid-class_input_node>` {
	public:
		// construction
	
		:ref:`InputROSNode<doxid-class_input_r_o_s_node_1a314f0fb99e37fee2949eff74850bfb78>`(const std::string& id);

		// methods
	
		virtual void :ref:`configure<doxid-class_input_r_o_s_node_1a424283f494808a16324a2a5f469bc55a>`();
	};

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
		virtual void :ref:`compute<doxid-class_input_node_1ab7d08881d8a20ed03a799be01dde46b5>`();
		void :ref:`registerOutput<doxid-class_input_node_1af15ec049b725a7e02f09650f81bfd97c>`(const std::string& id);
		:ref:`OutputPort<doxid-class_output_port>`<DATA>* :ref:`getSinglePort<doxid-class_input_node_1ad9a253c855c20d797820824b319d4a31>`(const std::string& id);
		:ref:`OutputPort<doxid-class_output_port>`<std::vector<const DATA*>>* :ref:`getListPort<doxid-class_input_node_1af2c401b83ae12585082add95df9d1270>`(const std::string& id);
		:ref:`InputNodePolicies::MsgPublishPolicy<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>` :ref:`msgPublishPolicy<doxid-class_input_node_1a4377e88c802c215e7e1174feca8e4735>`();
		:ref:`InputNodePolicies::MsgCachePolicy<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>` :ref:`msgCachePolicy<doxid-class_input_node_1ac7c8589e78d9b194e222edfab912bd66>`();
		void :ref:`setMsgPublishPolicy<doxid-class_input_node_1a6b9e5b9eaa7a8f1b275b698adacc765c>`(:ref:`InputNodePolicies::MsgPublishPolicy<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>` msgPublishPolicy);
		void :ref:`setMsgCachePolicy<doxid-class_input_node_1a7e2197a005d86ee5191598f6e13b45a5>`(:ref:`InputNodePolicies::MsgCachePolicy<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>` msgCachePolicy);

.. _details-class_input_r_o_s_node:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Input node used to connect a ROS subscriber to the computational graph.

The template parameter MSG_TYPE is the ROS msg type this node subscribes to

Construction
------------

.. index:: pair: function; InputROSNode
.. _doxid-class_input_r_o_s_node_1a314f0fb99e37fee2949eff74850bfb78:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	InputROSNode(const std::string& id)

Constructor.

Methods
-------

.. index:: pair: function; configure
.. _doxid-class_input_r_o_s_node_1a424283f494808a16324a2a5f469bc55a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void configure()

Configures the node making it ready to execute 'compute'.


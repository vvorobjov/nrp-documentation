.. index:: pair: class; ComputationalNode
.. _doxid-class_computational_node:

class ComputationalNode
=======================

.. toctree::
	:hidden:

	enum_ComputationalNode_NodeType.rst

Overview
~~~~~~~~

Base class implementing a node in the computation graph. :ref:`More...<details-class_computational_node>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <computational_node.h>
	
	class ComputationalNode {
	public:
		// enums
	
		enum :ref:`NodeType<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>`;

		// construction
	
		:target:`ComputationalNode<doxid-class_computational_node_1abeb548303f0988f49e04ed1471cfa4f6>`();
		:ref:`ComputationalNode<doxid-class_computational_node_1a668973b4f5901fa6dd59e33f354605eb>`(std::string id, :ref:`NodeType<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>` type);

		// methods
	
		bool :ref:`isVisited<doxid-class_computational_node_1aa657870e632df5c4ed11470d10e65d5f>`() const;
		void :ref:`setVisited<doxid-class_computational_node_1a4d75c538b8bbc2f9c26278d27ad8a434>`(bool visited);
		const std::string& :ref:`id<doxid-class_computational_node_1aeab0953471cf02647c0264c8b474afb5>`() const;
		:ref:`NodeType<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>` :ref:`type<doxid-class_computational_node_1a4cb10cde56ec02dd5e31c5b5498388ed>`() const;
		virtual void :ref:`configure<doxid-class_computational_node_1abf931072caba0154df7a388c515b43d8>`() = 0;
		virtual void :ref:`compute<doxid-class_computational_node_1a593ccb6e371475e628e11253b562a7a2>`() = 0;
	};

	// direct descendants

	template <typename... INPUT_TYPES, typename... OUTPUT_TYPES>
	class :ref:`FunctionalNode<std::tuple<INPUT_TYPES...>, std::tuple<OUTPUT_TYPES...>><doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7>`;

	template <class DATA>
	class :ref:`InputNode<doxid-class_input_node>`;

	template <class DATA>
	class :ref:`OutputNode<doxid-class_output_node>`;
.. _details-class_computational_node:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Base class implementing a node in the computation graph.

Construction
------------

.. index:: pair: function; ComputationalNode
.. _doxid-class_computational_node_1a668973b4f5901fa6dd59e33f354605eb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ComputationalNode(std::string id, :ref:`NodeType<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>` type)

Constructor.

Methods
-------

.. index:: pair: function; isVisited
.. _doxid-class_computational_node_1aa657870e632df5c4ed11470d10e65d5f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool isVisited() const

Returns true if the node has been marked as visited, false otherwise.

.. index:: pair: function; setVisited
.. _doxid-class_computational_node_1a4d75c538b8bbc2f9c26278d27ad8a434:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void setVisited(bool visited)

Sets a value for the node 'visited' property.

.. index:: pair: function; id
.. _doxid-class_computational_node_1aeab0953471cf02647c0264c8b474afb5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const std::string& id() const

Returns the node 'id'.

.. index:: pair: function; type
.. _doxid-class_computational_node_1a4cb10cde56ec02dd5e31c5b5498388ed:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`NodeType<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>` type() const

Returns the node 'type'.

.. index:: pair: function; configure
.. _doxid-class_computational_node_1abf931072caba0154df7a388c515b43d8:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void configure() = 0

Configures the node making it ready to execute 'compute'.

.. index:: pair: function; compute
.. _doxid-class_computational_node_1a593ccb6e371475e628e11253b562a7a2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void compute() = 0

Requests the node to execute its computation.


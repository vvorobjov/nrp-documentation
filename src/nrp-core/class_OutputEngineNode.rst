.. index:: pair: class; OutputEngineNode
.. _doxid-class_output_engine_node:

class OutputEngineNode
======================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Output node used to connect the computational graph with an :ref:`EngineClient <doxid-class_engine_client>`. :ref:`More...<details-class_output_engine_node>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <output_node.h>
	
	class OutputEngineNode: public :ref:`OutputNode<doxid-class_output_node>` {
	public:
		// typedefs
	
		typedef :ref:`DataPackInterface<doxid-class_data_pack_interface>`* :target:`DataPackInterfacePtr<doxid-class_output_engine_node_1a3f490229742a6305d26392c6a257bfa8>`;

		// construction
	
		:ref:`OutputEngineNode<doxid-class_output_engine_node_1a3249a6f9c939276af4d298dc69581265>`(const std::string& id, const std::string& engineName);

		// methods
	
		std::vector<:ref:`DataPackInterface<doxid-class_data_pack_interface>`*> :ref:`getDataPacks<doxid-class_output_engine_node_1a36e5a44ca3d5104b001c762c3f8cff7c>`();
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
		virtual void :ref:`configure<doxid-class_output_node_1ab38541a1582f99a453254c5ebec2449e>`();
		virtual void :ref:`compute<doxid-class_output_node_1a8bc64d2d817505fbfd68cc266ec7a9f0>`();
	
		template <class T_IN>
		:ref:`InputPort<doxid-class_input_port>`<T_IN, DATA>* :ref:`getOrRegisterInput<doxid-class_output_node_1a833d82862c1d613aab1448c3e0157237>`(const std::string& id);
	
		:ref:`OutputNodePolicies::MsgPublishPolicy<doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2>` :ref:`msgPublishPolicy<doxid-class_output_node_1a76f8b117583989b59d210f3c4759d765>`();

.. _details-class_output_engine_node:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Output node used to connect the computational graph with an :ref:`EngineClient <doxid-class_engine_client>`.

Construction
------------

.. index:: pair: function; OutputEngineNode
.. _doxid-class_output_engine_node_1a3249a6f9c939276af4d298dc69581265:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	OutputEngineNode(const std::string& id, const std::string& engineName)

Constructor

Methods
-------

.. index:: pair: function; getDataPacks
.. _doxid-class_output_engine_node_1a36e5a44ca3d5104b001c762c3f8cff7c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::vector<:ref:`DataPackInterface<doxid-class_data_pack_interface>`*> getDataPacks()

Returns all datapacks stored in the node and clears the cache


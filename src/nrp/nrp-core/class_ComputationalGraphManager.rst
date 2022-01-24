.. index:: pair: class; ComputationalGraphManager
.. _doxid-class_computational_graph_manager:

class ComputationalGraphManager
===============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Singleton class managing a computational graph. :ref:`More...<details-class_computational_graph_manager>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <computational_graph_manager.h>
	
	class ComputationalGraphManager {
	public:
		// construction
	
		:target:`ComputationalGraphManager<doxid-class_computational_graph_manager_1ab5836216f12fc1133a74b1aaf1017752>`(const ComputationalGraphManager&);
		:target:`ComputationalGraphManager<doxid-class_computational_graph_manager_1ac7fadcec9603cd0db6cdca6cd6e2ef70>`(ComputationalGraphManager&&);

		// methods
	
		ComputationalGraphManager& :target:`operator =<doxid-class_computational_graph_manager_1ab8f597dea4506d73b52c9c0ebf3c565d>` (const ComputationalGraphManager&);
		ComputationalGraphManager& :target:`operator =<doxid-class_computational_graph_manager_1a7fd7717fcb25a09afe4ac7fcf5286651>` (ComputationalGraphManager&&);
		void :ref:`registerNode<doxid-class_computational_graph_manager_1a7b38bb941eb386e6e0916c16cac42b1f>`(std::shared_ptr<:ref:`ComputationalNode<doxid-class_computational_node>`>& obj);
		:ref:`ComputationalNode<doxid-class_computational_node>`* :ref:`getNode<doxid-class_computational_graph_manager_1ae9cd43ae91337c6e03560b103e351576>`(const std::string& id);
	
		template <class T_IN, class T_OUT>
		void :ref:`registerEdge<doxid-class_computational_graph_manager_1a5d7431300f653e18148582567bf37245>`(
			:ref:`OutputPort<doxid-class_output_port>`<T_IN>* source,
			:ref:`InputPort<doxid-class_input_port>`<T_IN, T_OUT>* target
		);
	
		void :ref:`compute<doxid-class_computational_graph_manager_1a1b51f786dd259e77e05bf1c896fc0dd8>`();
		void :ref:`configure<doxid-class_computational_graph_manager_1adf61b9d197bbf95706861ce168b6efc8>`();
		void :ref:`clear<doxid-class_computational_graph_manager_1a79bd3ff2515d3187cda2b8352d923f10>`();
		static ComputationalGraphManager& :ref:`getInstance<doxid-class_computational_graph_manager_1a1b4d16e1a98a63d7708b441086b6fc38>`();
		static ComputationalGraphManager& :ref:`resetInstance<doxid-class_computational_graph_manager_1a52657671dec81d2e391ddbee0aa92839>`();
	};
.. _details-class_computational_graph_manager:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Singleton class managing a computational graph.

:ref:`ComputationalGraph <doxid-class_computational_graph>` is only concerned about the graph structure and node execution policies while :ref:`ComputationalGraphManager <doxid-class_computational_graph_manager>` remains responsible for managing the registration of nodes and edges, establishing the actual connections between ports. It owns a :ref:`ComputationalGraph <doxid-class_computational_graph>` and all its nodes.

Methods
-------

.. index:: pair: function; registerNode
.. _doxid-class_computational_graph_manager_1a7b38bb941eb386e6e0916c16cac42b1f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerNode(std::shared_ptr<:ref:`ComputationalNode<doxid-class_computational_node>`>& obj)

Register a node in the graph.

If the node is of type 'Functional', an attempt to register a node with an 'id' already existing is considered an error and an exception is thrown. If the type is different to 'Functional' the new node is not registered and the reference obj is shifted to the existing node with the same 'id'

.. index:: pair: function; getNode
.. _doxid-class_computational_graph_manager_1ae9cd43ae91337c6e03560b103e351576:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`ComputationalNode<doxid-class_computational_node>`* getNode(const std::string& id)

Retrieve a node from the graph as a pointer.

.. index:: pair: function; registerEdge
.. _doxid-class_computational_graph_manager_1a5d7431300f653e18148582567bf37245:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class T_IN, class T_OUT>
	void registerEdge(
		:ref:`OutputPort<doxid-class_output_port>`<T_IN>* source,
		:ref:`InputPort<doxid-class_input_port>`<T_IN, T_OUT>* target
	)

Connects an :ref:`InputPort <doxid-class_input_port>` to an Output port and registers an edge in the graph between their parent nodes.

.. index:: pair: function; compute
.. _doxid-class_computational_graph_manager_1a1b51f786dd259e77e05bf1c896fc0dd8:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void compute()

Executes :ref:`ComputationalGraph <doxid-class_computational_graph>`.

.. index:: pair: function; configure
.. _doxid-class_computational_graph_manager_1adf61b9d197bbf95706861ce168b6efc8:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void configure()

Configure :ref:`ComputationalGraph <doxid-class_computational_graph>`.

.. index:: pair: function; clear
.. _doxid-class_computational_graph_manager_1a79bd3ff2515d3187cda2b8352d923f10:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void clear()

Resets :ref:`ComputationalGraphManager <doxid-class_computational_graph_manager>`.

.. index:: pair: function; getInstance
.. _doxid-class_computational_graph_manager_1a1b4d16e1a98a63d7708b441086b6fc38:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static ComputationalGraphManager& getInstance()

Get singleton instance of :ref:`ComputationalGraphManager <doxid-class_computational_graph_manager>`.

.. index:: pair: function; resetInstance
.. _doxid-class_computational_graph_manager_1a52657671dec81d2e391ddbee0aa92839:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static ComputationalGraphManager& resetInstance()

Reset singleton instance.


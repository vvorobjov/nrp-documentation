.. index:: pair: class; SimpleInputEdge
.. _doxid-class_simple_input_edge:

template class SimpleInputEdge
==============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Helper template class used to implement Python input edge decorators. :ref:`More...<details-class_simple_input_edge>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <input_edge.h>
	
	template <class T_IN, INPUT_C<T_IN> INPUT_CLASS>
	class SimpleInputEdge {
	public:
		// construction
	
		:target:`SimpleInputEdge<doxid-class_simple_input_edge_1aca2d6a3ce7d2ac32eb9dffe283c46723>`();
	
		:ref:`SimpleInputEdge<doxid-class_simple_input_edge_1ad29908f73ca06788720ef62908647d3c>`(
			std::string keyword,
			std::string id,
			std::string port,
			:ref:`InputNodePolicies::MsgPublishPolicy<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>` msgPublishPolicy,
			:ref:`InputNodePolicies::MsgCachePolicy<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>` msgCachePolicy
		);

		// methods
	
		boost::python::object :ref:`pySetup<doxid-class_simple_input_edge_1a9b24aa19aa3659abba9c173b2738cf5b>`(const boost::python::object& obj);
	};

	// direct descendants

	class :ref:`InputDummyEdge<doxid-class_input_dummy_edge>`;
	class :ref:`InputEngineEdge<doxid-class_input_engine_edge>`;

	template <class MSG_TYPE>
	class :ref:`InputROSEdge<doxid-class_input_r_o_s_edge>`;
.. _details-class_simple_input_edge:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Helper template class used to implement Python input edge decorators.

Construction
------------

.. index:: pair: function; SimpleInputEdge
.. _doxid-class_simple_input_edge_1ad29908f73ca06788720ef62908647d3c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	SimpleInputEdge(
		std::string keyword,
		std::string id,
		std::string port,
		:ref:`InputNodePolicies::MsgPublishPolicy<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>` msgPublishPolicy,
		:ref:`InputNodePolicies::MsgCachePolicy<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>` msgCachePolicy
	)

Constructor.

Methods
-------

.. index:: pair: function; pySetup
.. _doxid-class_simple_input_edge_1a9b24aa19aa3659abba9c173b2738cf5b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object pySetup(const boost::python::object& obj)

**call** function in the decorator

It creates and registers an input node. Afterwards add a port to it and registers an edge to 'obj'. 'obj' is expected to be a Python object wrapping a :ref:`PythonFunctionalNode <doxid-class_python_functional_node>`


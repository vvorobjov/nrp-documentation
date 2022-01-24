.. index:: pair: class; InputEngineEdge
.. _doxid-class_input_engine_edge:

class InputEngineEdge
=====================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <input_node.h>
	
	class InputEngineEdge: public :ref:`SimpleInputEdge<doxid-class_simple_input_edge>` {
	public:
		// construction
	
		:target:`InputEngineEdge<doxid-class_input_engine_edge_1ab30f7d4354df7d280782e4dedf59988f>`(
			const std::string& keyword,
			const std::string& address,
			:ref:`InputNodePolicies::MsgCachePolicy<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>` msgCachePolicy
		);
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		boost::python::object :ref:`pySetup<doxid-class_simple_input_edge_1a9b24aa19aa3659abba9c173b2738cf5b>`(const boost::python::object& obj);


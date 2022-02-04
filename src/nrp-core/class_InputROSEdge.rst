.. index:: pair: class; InputROSEdge
.. _doxid-class_input_r_o_s_edge:

template class InputROSEdge
===========================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <input_node.h>
	
	template <class MSG_TYPE>
	class InputROSEdge: public :ref:`SimpleInputEdge<doxid-class_simple_input_edge>` {
	public:
		// construction
	
		:target:`InputROSEdge<doxid-class_input_r_o_s_edge_1ad8f58c27688a1ee1ea4ec20bc6eb680f>`(
			const std::string& keyword,
			const std::string& address,
			:ref:`InputNodePolicies::MsgPublishPolicy<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>` msgPublishPolicy,
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


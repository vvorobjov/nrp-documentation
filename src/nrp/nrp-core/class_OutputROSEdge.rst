.. index:: pair: class; OutputROSEdge
.. _doxid-class_output_r_o_s_edge:

template class OutputROSEdge
============================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <output_node.h>
	
	template <class MSG_TYPE>
	class OutputROSEdge: public :ref:`SimpleOutputEdge<doxid-class_simple_output_edge>` {
	public:
		// construction
	
		:target:`OutputROSEdge<doxid-class_output_r_o_s_edge_1ae174592e64d3c4a30021fd026d64e18b>`(const std::string& keyword, const std::string& address);
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		boost::python::object :ref:`pySetup<doxid-class_simple_output_edge_1ac9393b03ca5a6708fec5b48936210ab3>`(const boost::python::object& obj);


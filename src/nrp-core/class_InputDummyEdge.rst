.. index:: pair: class; InputDummyEdge
.. _doxid-class_input_dummy_edge:

class InputDummyEdge
====================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <input_dummy.h>
	
	class InputDummyEdge: public :ref:`SimpleInputEdge<doxid-class_simple_input_edge>` {
	public:
		// construction
	
		:target:`InputDummyEdge<doxid-class_input_dummy_edge_1a27c18e7a123214ffd90c9dd241ec7e68>`(
			const std::string& keyword,
			const std::string& id,
			boost::python::object value
		);
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		boost::python::object :ref:`pySetup<doxid-class_simple_input_edge_1a9b24aa19aa3659abba9c173b2738cf5b>`(const boost::python::object& obj);


.. index:: pair: class; SimpleOutputEdge
.. _doxid-class_simple_output_edge:

template class SimpleOutputEdge
===============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Helper template class used to implement Python output edge decorators. :ref:`More...<details-class_simple_output_edge>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <output_edge.h>
	
	template <class T_OUT, OUTPUT_C<T_OUT> OUTPUT_CLASS>
	class SimpleOutputEdge {
	public:
		// construction
	
		:target:`SimpleOutputEdge<doxid-class_simple_output_edge_1a75e1135ba24e14d34c4f3a6f289edbb1>`();
		:ref:`SimpleOutputEdge<doxid-class_simple_output_edge_1ac13e691fffe7df6544abb6f989e44bb5>`(std::string keyword, std::string id, std::string port);

		// methods
	
		boost::python::object :ref:`pySetup<doxid-class_simple_output_edge_1ac9393b03ca5a6708fec5b48936210ab3>`(const boost::python::object& obj);
	};

	// direct descendants

	class :ref:`OutputDummyEdge<doxid-class_output_dummy_edge>`;
	class :ref:`OutputEngineEdge<doxid-class_output_engine_edge>`;

	template <class MSG_TYPE>
	class :ref:`OutputROSEdge<doxid-class_output_r_o_s_edge>`;
.. _details-class_simple_output_edge:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Helper template class used to implement Python output edge decorators.

Construction
------------

.. index:: pair: function; SimpleOutputEdge
.. _doxid-class_simple_output_edge_1ac13e691fffe7df6544abb6f989e44bb5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	SimpleOutputEdge(std::string keyword, std::string id, std::string port)

Constructor.

Methods
-------

.. index:: pair: function; pySetup
.. _doxid-class_simple_output_edge_1ac9393b03ca5a6708fec5b48936210ab3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object pySetup(const boost::python::object& obj)

**call** function in the decorator

It creates and registers an output node. Afterwards add a port to it and registers an edge from 'obj'. 'obj' is expected to be a Python object wrapping a :ref:`PythonFunctionalNode <doxid-class_python_functional_node>`


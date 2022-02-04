.. index:: pair: class; F2FEdge
.. _doxid-class_f2_f_edge:

class F2FEdge
=============

.. toctree::
	:hidden:

Overview
~~~~~~~~

Helper class used to implement a :ref:`F2FEdge <doxid-class_f2_f_edge>` Python decorator. :ref:`More...<details-class_f2_f_edge>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <functional_node.h>
	
	class F2FEdge {
	public:
		// construction
	
		:target:`F2FEdge<doxid-class_f2_f_edge_1a0dbebb63218b566cdff4e72276327bd4>`(const std::string& keyword, const std::string& address);

		// methods
	
		boost::python::object :ref:`pySetup<doxid-class_f2_f_edge_1aa6ed9adb0a28777eac15a1c392984a5f>`(const boost::python::object& obj);
	};
.. _details-class_f2_f_edge:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Helper class used to implement a :ref:`F2FEdge <doxid-class_f2_f_edge>` Python decorator.

Methods
-------

.. index:: pair: function; pySetup
.. _doxid-class_f2_f_edge_1aa6ed9adb0a28777eac15a1c392984a5f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object pySetup(const boost::python::object& obj)

**call** function in the decorator

It receives a Python object wrapping a :ref:`PythonFunctionalNode <doxid-class_python_functional_node>` and add an :ref:`F2FEdge <doxid-class_f2_f_edge>` to it


.. index:: pair: class; PythonFunctionalNode
.. _doxid-class_python_functional_node:

class PythonFunctionalNode
==========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Specialization of :ref:`FunctionalNode <doxid-class_functional_node>` in which _function is a python callable object. :ref:`More...<details-class_python_functional_node>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <functional_node.h>
	
	class PythonFunctionalNode: public :ref:`FunctionalNode<doxid-class_functional_node>` {
	public:
		// construction
	
		:ref:`PythonFunctionalNode<doxid-class_python_functional_node_1a15032c7991e2d8f0d8bc022eb4e323fa>`(
			const std::string& id,
			const boost::python::list& o_ports,
			:ref:`FunctionalNodePolicies::ExecutionPolicy<doxid-namespace_functional_node_policies_1a3f872dbefb885b0dca2745e76e002b87>` policy = FunctionalNodePolicies::ExecutionPolicy::ON_NEW_INPUT
		);

		// methods
	
		void :ref:`configure<doxid-class_python_functional_node_1a0e2d0efe053b5edfa9d57ecf38476938>`();
		boost::python::object :ref:`pySetup<doxid-class_python_functional_node_1a083f8917b190580912c9acaed2e6bc2d>`(boost::python::object obj);
	
		template <typename T_IN, size_t N = 0>
		:ref:`InputPort<doxid-class_input_port>`<T_IN, bpy::object>* :ref:`getOrRegisterInput<doxid-class_python_functional_node_1a34606236035e415d8e2018c9e15105bc>`(const std::string& id);
	
		template <size_t N = 0>
		:ref:`OutputPort<doxid-class_output_port>`<bpy::object>* :ref:`registerOutput<doxid-class_python_functional_node_1a0a2fd6287f10f12fa7cdcbf5b5167b94>`(const std::string& id);
	
		template <size_t N = 0>
		:ref:`OutputPort<doxid-class_output_port>`<bpy::object>* :ref:`getOutput<doxid-class_python_functional_node_1ac7eb61caf5b170cf919ac97efe1616d4>`(const std::string& id);
	
		void :ref:`registerF2FEdge<doxid-class_python_functional_node_1a3b5d34aa3e8458687edda4e610d4636e>`(const std::string& i_port, const std::string& address);
	};
.. _details-class_python_functional_node:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Specialization of :ref:`FunctionalNode <doxid-class_functional_node>` in which _function is a python callable object.

_function can have a variable number of inputs and outputs between 0 and input_s/output_s

Construction
------------

.. index:: pair: function; PythonFunctionalNode
.. _doxid-class_python_functional_node_1a15032c7991e2d8f0d8bc022eb4e323fa:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	PythonFunctionalNode(
		const std::string& id,
		const boost::python::list& o_ports,
		:ref:`FunctionalNodePolicies::ExecutionPolicy<doxid-namespace_functional_node_policies_1a3f872dbefb885b0dca2745e76e002b87>` policy = FunctionalNodePolicies::ExecutionPolicy::ON_NEW_INPUT
	)

Constructor.

Methods
-------

.. index:: pair: function; configure
.. _doxid-class_python_functional_node_1a0e2d0efe053b5edfa9d57ecf38476938:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void configure()

Configure this node.

.. index:: pair: function; pySetup
.. _doxid-class_python_functional_node_1a083f8917b190580912c9acaed2e6bc2d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object pySetup(boost::python::object obj)

Setup this node with a python callable object and registers it to :ref:`ComputationalGraphManager <doxid-class_computational_graph_manager>`.

After calling this function the object is moved into a shared pointer which is returned by the function. Afterwards this object shouldn't be used. Use the returned shared pointer.

It is meant to be used as **call** function in a Python decorator which creates a :ref:`PythonFunctionalNode <doxid-class_python_functional_node>` from a given Python function

.. index:: pair: function; getOrRegisterInput
.. _doxid-class_python_functional_node_1a34606236035e415d8e2018c9e15105bc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename T_IN, size_t N = 0>
	:ref:`InputPort<doxid-class_input_port>`<T_IN, bpy::object>* getOrRegisterInput(const std::string& id)

Safely get or register an input port.

.. index:: pair: function; registerOutput
.. _doxid-class_python_functional_node_1a0a2fd6287f10f12fa7cdcbf5b5167b94:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <size_t N = 0>
	:ref:`OutputPort<doxid-class_output_port>`<bpy::object>* registerOutput(const std::string& id)

Safely registers an output port.

.. index:: pair: function; getOutput
.. _doxid-class_python_functional_node_1ac7eb61caf5b170cf919ac97efe1616d4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <size_t N = 0>
	:ref:`OutputPort<doxid-class_output_port>`<bpy::object>* getOutput(const std::string& id)

Safely get an output port.

.. index:: pair: function; registerF2FEdge
.. _doxid-class_python_functional_node_1a3b5d34aa3e8458687edda4e610d4636e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerF2FEdge(const std::string& i_port, const std::string& address)

Request the registration of an edge between an output port in another functional node an i_port input port in this node.


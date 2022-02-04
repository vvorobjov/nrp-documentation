.. index:: pair: class; InputPort
.. _doxid-class_input_port:

template class InputPort
========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Implementation of an input port in the computation graph. :ref:`More...<details-class_input_port>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <input_port.h>
	
	template <class T_IN, class T_OUT>
	class InputPort: public :ref:`Port<doxid-class_port>` {
	public:
		// construction
	
		:ref:`InputPort<doxid-class_input_port_1ada31d39322d14b502955f4081befba60>`(
			const std::string& id,
			:ref:`ComputationalNode<doxid-class_computational_node>`* parent,
			std::function<void(const T_OUT*)> callback,
			std::size_t maxSubs = 0
		);

		// methods
	
		void :ref:`subscribeTo<doxid-class_input_port_1a6965c72cc3bfe39bbcc788acdc2ba3c5>`(:ref:`OutputPort<doxid-class_output_port>`<T_IN>* port);
		virtual size_t :ref:`subscriptionsSize<doxid-class_input_port_1ab72890684b89b1fe7232e28c30e457d5>`();
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		const std::string& :ref:`id<doxid-class_port_1adbf5fd7698844b5083f6d07ac9ee8dc7>`();
		:ref:`ComputationalNode<doxid-class_computational_node>`* :ref:`parent<doxid-class_port_1abcc7c30341c5fed40e37fc34b3a66c9c>`() const;
		virtual size_t :ref:`subscriptionsSize<doxid-class_port_1a192e14894419e0ad363743b7a1523dd8>`() = 0;

.. _details-class_input_port:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Implementation of an input port in the computation graph.

It converts and passes incoming msgs using a callback function

Construction
------------

.. index:: pair: function; InputPort
.. _doxid-class_input_port_1ada31d39322d14b502955f4081befba60:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	InputPort(
		const std::string& id,
		:ref:`ComputationalNode<doxid-class_computational_node>`* parent,
		std::function<void(const T_OUT*)> callback,
		std::size_t maxSubs = 0
	)

Constructor.

Methods
-------

.. index:: pair: function; subscribeTo
.. _doxid-class_input_port_1a6965c72cc3bfe39bbcc788acdc2ba3c5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void subscribeTo(:ref:`OutputPort<doxid-class_output_port>`<T_IN>* port)

Subscribes this port to an :ref:`OutputPort <doxid-class_output_port>` 'port'.

.. index:: pair: function; subscriptionsSize
.. _doxid-class_input_port_1ab72890684b89b1fe7232e28c30e457d5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual size_t subscriptionsSize()

Return the number ports this port is subscribed to.


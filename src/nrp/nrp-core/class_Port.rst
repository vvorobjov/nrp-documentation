.. index:: pair: class; Port
.. _doxid-class_port:

class Port
==========

.. toctree::
	:hidden:

Overview
~~~~~~~~

Base class implementing a port in the computational graph. :ref:`More...<details-class_port>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <port.h>
	
	class Port {
	public:
		// construction
	
		:target:`Port<doxid-class_port_1a58a3d6b928a52a2ecfb0c1d1b655d34d>`();
		:ref:`Port<doxid-class_port_1a3a60b4bd2afa10b11a2555dd1a2fb5b3>`(std::string id, :ref:`ComputationalNode<doxid-class_computational_node>`* parent);

		// methods
	
		const std::string& :ref:`id<doxid-class_port_1adbf5fd7698844b5083f6d07ac9ee8dc7>`();
		:ref:`ComputationalNode<doxid-class_computational_node>`* :ref:`parent<doxid-class_port_1abcc7c30341c5fed40e37fc34b3a66c9c>`() const;
		virtual size_t :ref:`subscriptionsSize<doxid-class_port_1a192e14894419e0ad363743b7a1523dd8>`() = 0;
	};

	// direct descendants

	template <class T_IN, class T_OUT>
	class :ref:`InputPort<doxid-class_input_port>`;

	template <class T>
	class :ref:`OutputPort<doxid-class_output_port>`;
.. _details-class_port:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Base class implementing a port in the computational graph.

Construction
------------

.. index:: pair: function; Port
.. _doxid-class_port_1a3a60b4bd2afa10b11a2555dd1a2fb5b3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	Port(std::string id, :ref:`ComputationalNode<doxid-class_computational_node>`* parent)

Constructor.

Methods
-------

.. index:: pair: function; id
.. _doxid-class_port_1adbf5fd7698844b5083f6d07ac9ee8dc7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const std::string& id()

Returns the port 'id'.

.. index:: pair: function; parent
.. _doxid-class_port_1abcc7c30341c5fed40e37fc34b3a66c9c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`ComputationalNode<doxid-class_computational_node>`* parent() const

Returns the port parent node.

.. index:: pair: function; subscriptionsSize
.. _doxid-class_port_1a192e14894419e0ad363743b7a1523dd8:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual size_t subscriptionsSize() = 0

Return the number of subscriptions of this port.

This is the number of ports this port is subscribed to in the case of input ports or the number of ports subscribed to this port in the case of output ports


.. index:: pair: class; OutputPort
.. _doxid-class_output_port:

template class OutputPort
=========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Implementation of an output port in the computation graph. :ref:`More...<details-class_output_port>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <output_port.h>
	
	template <class T>
	class OutputPort: public :ref:`Port<doxid-class_port>` {
	public:
		// construction
	
		:ref:`OutputPort<doxid-class_output_port_1ae78bbf100b863a32183a1c394dddbff6>`(const std::string& id, :ref:`ComputationalNode<doxid-class_computational_node>`* parent);

		// methods
	
		void :ref:`publish<doxid-class_output_port_1a67f02c8a60ae4a81224cce87587f6b52>`(const T* msg);
		virtual size_t :ref:`subscriptionsSize<doxid-class_output_port_1a51b67e80feb92845981f36eab29190c0>`();
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

.. _details-class_output_port:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Implementation of an output port in the computation graph.

It forwards msgs to subscribed ports via its 'publish' method

Construction
------------

.. index:: pair: function; OutputPort
.. _doxid-class_output_port_1ae78bbf100b863a32183a1c394dddbff6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	OutputPort(const std::string& id, :ref:`ComputationalNode<doxid-class_computational_node>`* parent)

Constructor.

Methods
-------

.. index:: pair: function; publish
.. _doxid-class_output_port_1a67f02c8a60ae4a81224cce87587f6b52:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void publish(const T* msg)

Publish a msg to all subscribers.

.. index:: pair: function; subscriptionsSize
.. _doxid-class_output_port_1a51b67e80feb92845981f36eab29190c0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual size_t subscriptionsSize()

Return the number the number of ports subscribed to this port.

